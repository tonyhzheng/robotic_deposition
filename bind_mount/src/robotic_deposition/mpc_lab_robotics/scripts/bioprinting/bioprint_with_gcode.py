#!/usr/bin/env python3 
"""
Class for controlling a ur5e robot arm to bioprint using a pneumatic-actuated needle
and g-code for a desired shape

Author: Tony Zheng, Jannik Pruessmann
"""
from mpc_lab_robotics.scripts.general.baseline_arm_controller import *
from mpc_lab_robotics.utils.utils import *
from mpc_lab_robotics.controllers.PID import PID

from std_msgs.msg import Float32MultiArray ,Float64,String,Int64,Float64MultiArray,UInt16
from keyboard.msg import Key

from gcodeparser import GcodeParser

import matplotlib
matplotlib.use('TkAgg') #'TkAgg'
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import csv 

import timeit
from timeit import Timer
np.set_printoptions(suppress=True,precision=5,linewidth = np.inf)

class controller_node(baseline_arm_controller):
    def __init__(self, frequency = 125, use_camera = False):
        super().__init__(frequency = frequency, use_camera = use_camera)
        self.run_main_loop()

    def initialize_additional_functions(self):
        self.define_additional_params()
        self.initialize_additional_ros()

    def define_additional_params(self):
        self.min_psi = 0
        self.max_psi = 120
        self.max_feedrate = 10000
        self.target_psi = 0
        self.home_pose = np.array([-0.74628,  0.06942,  0.1053,  3.14155, -0.00005, -0.00004]) + [0,0,0.1,0,0,0] #@todo: determine reasonable home position (where the syringe can be loaded and tubing can be connected)

        # flags
        self.extrusion_enabled = True   # indicates if the áir pressure/extrusion is currently enabled
        self.extruding = False          # indicates if the system is currently intended to be extruding
        self.plot_printed_lines_enabled = False # indicates if the matplotlib plot of printed lines is currently enabled
        self.scale_data_received = False        # indicates if data from the weight scale was received successfully
        self.defect_detected = False            # when using the camera feedback: indicates if the current printing position is known to be defective

        #defect detection
        self.defect_startpoints = []
        self.defect_start_timestamps = []
        self.defect_endpoints = []
        self.defect_end_timestamps = []
        self.nozzle_meas_point_offset = np.array([0, 0.0045,0,0,0,0]) #m positional offset between measurement point and needle tip + time offset due to stabilisation over 15 frames

        #transformation from robot position to needle tip
        self.ee_to_needletip_transform = np.array([0.0, 0.065, 0.130,1]).reshape(4,1)
        self.needletip_to_ee_transform = np.array([0.0, -0.065, -0.130,1]).reshape(4,1)

        self.ee_to_needletip_delta = np.array([0.0, -0.063, 0.123])
        self.ee_to_needletip_delta = np.array([0.0, -0.063, 0.127])
        self.needletip_to_ee_delta = -self.ee_to_needletip_delta
        
        self.ee_to_needletip_transform = np.hstack((self.ee_to_needletip_delta,1)).reshape(4,1)
        self.needletip_to_ee_transform = np.hstack((self.needletip_to_ee_delta,1)).reshape(4,1)
 
        #init PID controller
        self.P = 20
        self.D = 0
        self.I = 0
        self.max_integrator = 0.005   
        self.min_integrator = -0.005
        self.width_control = PID(P = self.P, D = self.D, I = self.I, max_integrator = self.max_integrator , min_integrator = self.min_integrator )
        self.set_psi = 0
        self.current_line_width = 0
        self.width_error = 0


    def stop_controller(self, *args, **kwargs):
        print("Stopping controller and stopping psi")
        self.send_input_to_ur5e("analog_value_update", 0)
        sys.exit(0)

    def initialize_additional_ros(self):
        rospy.Subscriber('scale_values', Float64,  self.callback_scale, queue_size = 1) 
        rospy.Subscriber('detected_line_width', Float64,  self.callback_line_width, queue_size = 1)
        rospy.Subscriber('keyboard/keydown', Key,  self.callback_keydown, queue_size = 10)

        
    def move_to_initial_pose(self,speed=0.05): 
        print("Moving to initial position") 

        
        self.pose_home = np.array([-0.41188,  0.1332 ,  0.41325, -3.14157,  0.00004,  0.     ])
        # self.pose_home = np.array([-0.41188,  0.13319,  0.41316, -2.22148,  2.2214 , -0.00018])
        self.joint_q_home = np.array([-3.14202, -1.39675, -1.91903, -1.39837,  1.57241,  1.57128])
        # self.joint_q_home = np.array([-3.14201, -1.39681, -1.91919, -1.39823,  1.57251,  3.14203])

        self.stay_at_initial = False
        # self.stay_at_initial = True

        total_time = self.calc_move_duration(self.pose_home, self.ur5e_actual_state.get_tcp_pose(), speed)
        self.send_input_to_ur5e("movej",self.joint_q_home , 0.5, 0.5, total_time, 0)
        self.wait_for_steady()

    def callback_scale(self,msg):
        self.scale_value = msg
        self.scale_data_received = True

    def callback_line_width(self,msg):
        self.current_line_width = msg.data

    def callback_keydown(self,msg):   
        if msg.code == 115: #if 's' is pressed: disable extrusion
            #stops and disables the extrusion immediately 
            print('Extrusion disabled by key, press "e" in key-window to enable')
            self.extrusion_enabled = False
            self.stop_extrusion(delay=0)
        elif msg.code == 101: #if 'e' is pressed: enable extrusion
            #enables to start extrusion and starts extrusion immediately if self.extruding == True
            print('Extrusion enabled by key, press "s" in key-window to disable')
            self.extrusion_enabled = True
            if self.extruding == True:
                self.start_extrusion(self.target_psi)


    def send_psi(self, psi):
        # DIN-Mount Precision Compressed Air Regulator, Electronic Ctrlled, 3-120 PSI, 4-20 mA, 1/4 NPT Female
        val = (psi-3)/117
        # self.analog_input_publisher.publish(np.clip(val,0,1))
        # self.send_input_to_ur5e("analog", np.clip(val,0,1))
        self.send_input_to_ur5e("analog_value_update", np.clip(val,0,1))

    def stop_extrusion(self, delay = 0.0):
        #delay in seconds
        if self.extruding == True:
            print('stopping extrusion')
            time.sleep(delay)
        self.send_psi(0)
        # self.send_input_to_ur5e("analog", [0])
        self.send_psi(0) #repeated to avoid command not getting send
        # self.send_input_to_ur5e("analog", [0])
        #@todo: implement feedback: get feedback from robot if analog_out ==4mA or use other pressure regulator with feedback of current pressure
        self.extruding = False

    def start_extrusion(self,psi = None, delay = 0.0):
        #delay in seconds
        if psi == None:
            psi = self.target_psi

        if self.extrusion_enabled == True:
            self.send_psi(psi)
            self.target_psi = psi
            # self.send_input_to_ur5e("analog", psi)
        else:
            self.send_psi(0)
            # self.send_input_to_ur5e("analog", 0)

        
        if self.extruding == False:
            # if extrusion was turned off before: wait system deadtime/delay
            time.sleep(delay)
            if self.extrusion_enabled == False:
                print('Startting extrusion: Extrusion is disabled')
            else:
                print('starting extrusion')
        self.extruding = True

    def set_feedrate_to_psi(self, feedrate):
        '''idea: use the feedrate given from G-code to set suitable air pressure
        currently unused as feedrate to pressure ratio is not well determined'''

        print('change feedrate to ' f"{feedrate}")
        try:
            self.target_psi = (feedrate/self.max_feedrate)*self.max_psi
        except:
            print('unable to set new airpressure')
        if self.target_psi > self.max_psi:
            self.target_psi = 1
            print('psi cannot exceed ' f"{self.max_psi}")
        elif self.target_psi < self.min_psi:
            self.target_psi = 0
            print('pressure signal cannot be below ' f"{self.min_psi}")

    def update_set_psi(self, reference_value):
        # read self.current_line_width (subscriber to camera node topic)
        # solve PID with last reference value = target_line_width, current value = self.current_line_width
        read_line_width = self.current_line_width

        if read_line_width <= 0 and self.defect_detected == False:
            self.defect_detected = True
            self.defect_startpoints.append(np.array(self.ur5e_actual_state.get_tcp_pose() + self.nozzle_meas_point_offset))
            print('detected new defect start at point ', self.defect_startpoints[-1])
        elif read_line_width > 0 and self.defect_detected == True:
            self.defect_detected = False
            self.defect_endpoints.append(np.array(self.ur5e_actual_state.get_tcp_pose() + self.nozzle_meas_point_offset))
            print('detected new defect end at point ', self.defect_endpoints[-1])

        if self.extrusion_enabled == False:
            return
        actuator_signal = self.width_control.solve(reference_value, read_line_width, 1/self.frequency)
        self.width_error = reference_value - read_line_width
        self.set_psi= self.set_psi + actuator_signal

    def clear_defect_list(self):
        '''clears the list of detected defect positions (start- and endpoints and corresponding timestamps)'''
        self.defect_startpoints = []
        self.defect_start_timestamps = []
        self.defect_endpoints = []
        self.defect_end_timestamps = []
 
    def make_linear_print_move(self, last_pose, new_pose, v = 0.05, psi_max = 0, psi_min=0, psi_trajectory='constant', retract=False, ax=None):
        '''this function calculates the time needed for the movement based on the target velocity.
        the movement path is divided in sections depending on the time needed, to get movement targets fo eacht timestep,
        updating the set air pressure for each step
        psi_trajectories: 'constant': constant air pressure psi_max, 'changing2': alternate air pressures psi_min
        and psi_max every 2 centimeters of movement , 'double_ramp': ramping down the pressure from psi_max to psi_min
        in the first half of movement and ramping it back up to psi_max in the second half of movement
        '''

        # calculate time needed for movement
        total_time = self.calc_move_duration(last_pose, new_pose, v) #get time needed for whole movement

        #clear defect list
        self.clear_defect_list()

        if psi_trajectory == 'changing2': # change pressure between psi_max and psi_min every two centimeters
            time_2cm = 0.02/v
            #print(time_2cm)
            intervall = 0
            psi = psi_min
            
        timestamp = time.time()
        t = time.time() - timestamp
        while t < total_time:
            t = time.time() - timestamp

            # calculate psi for next movement step
            if psi_trajectory == 'constant':
                psi = psi_max
            elif psi_trajectory == 'double_ramp':
                if t <= total_time/2:
                    psi = psi_max - (psi_max-psi_min)*2*t/total_time 
                else:
                    psi = psi_min + (psi_max-psi_min)*(2*t - total_time)/total_time
            elif psi_trajectory == 'changing2': # change pressure between psi_max and psi_min every two centimeters
                if t > (intervall * time_2cm):
                    intervall += 1
                    if intervall % 2 == 0:
                        psi = psi_min
                    else:
                        psi = psi_max
                    print('changing pressure to ',psi,'psi')

            self.start_extrusion(psi=psi,delay=0)

            #get next movement section based on timing information
            offset = t*(new_pose-last_pose)/total_time 
            next_step = last_pose + offset 
            self.send_input_to_ur5e("servol", next_step, [0.1,0.1,1.0/125,0.05,1200])
            self.rate.sleep()

        self.stop_extrusion(delay = 0)
        current_pose = new_pose
        #for plotting printed lines   
        if self.plot_printed_lines_enabled == True and ax != None:    
            if self.extruding == True:
                ax.plot([last_pose[0],new_pose[0]],[last_pose[1],new_pose[1]],[last_pose[2],new_pose[2]])
                plt.pause(0.05)

        if retract == True:
            current_pose = self.make_movej_move(current_pose, current_pose + [0,0,0.05,0,0,0], v=0.05)
        return current_pose

    def make_width_controlled_print_move(self, last_pose, new_pose, target_line_width, v = 0.02, fill_defects=False, retract=False, ax=None):
        '''this function calculates the time needed for the movement based on the target velocity.
        the movement path is divided in sections depending on the time needed, to get movement targets fo eacht timestep,
        updating the set air pressure using a PID controller and visual feedback for each step'''

        # calculate time needed for movement
        total_time = self.calc_move_duration(last_pose, new_pose, v) #get time needed for whole movement
        self.set_psi = 30 # choose depending on needle inner diameter (e.g. 30 for 0.42 needle)

        #clear defect list
        self.clear_defect_list()
        
        
        with open(os.path.dirname(os.path.abspath(__file__))+'/controller_log.csv', 'w', newline='') as f:
            # create csv writer
            writer = csv.writer(f)
            command_is_extruding = False

            #write settings
            writer.writerow(['Controller Settings: P,D,I,max_integrator, min_integrator', self.P, self.D, self.I, self.max_integrator, self.min_integrator])
            writer.writerow(['time [s]','XYZ [m]', 'velocity [m/s]', 'current line width [m]', 'target line width [m]', 'width error [m]', 'pressure [psi]'])

            timestamp = time.time()
            t = time.time() - timestamp
            while t < total_time:
                t = time.time() - timestamp #for offline t +=0.008 and initial as zero

                # calculate psi for next movement step
                self.update_set_psi(target_line_width)
                psi = self.set_psi
                self.start_extrusion(psi=psi,delay=0)

                writer.writerow([t,self.ur5e_actual_state.get_tcp_pose(), v, self.current_line_width, target_line_width, self.width_error, psi])
                #get next movement section based on timing information
                offset = t*(new_pose-last_pose)/total_time 
                next_step = last_pose + offset 
                self.send_input_to_ur5e("servol", next_step, [0.1,0.1,1.0/125,0.05,1200])
                self.rate.sleep()

        self.stop_extrusion(delay = 0)
        current_pose = new_pose
        if self.defect_detected == True:
            self.defect_endpoints.append(np.array(new_pose)) #set target pose as endpoint for last defect

        #for plotting printed lines   
        if self.plot_printed_lines_enabled == True and ax != None:    
            if self.extruding == True:
                ax.plot([last_pose[0],new_pose[0]],[last_pose[1],new_pose[1]],[last_pose[2],new_pose[2]])
                plt.pause(0.05)

        if fill_defects == True:
            # check if defects were detected and go to each defect to fill it
            # implemented by now only for fixed/'constant' psi
            if len(self.defect_startpoints) > 1: # exclude identification as defect at beginning
                self.defect_startpoints.pop(0)
                self.defect_endpoints.pop(0)
                for i,startpoint in enumerate(self.defect_startpoints):
                    print('go to defect startpoint number ', i, 'at ', startpoint )
                    # retract 1cm
                    current_pose = self.make_movej_move(current_pose, current_pose + [0,0,0.01,0,0,0], v=0.05)
                    # go above startpoint
                    current_pose = self.make_movej_move(current_pose, startpoint + [0,0,0.01,0,0,0], v=0.05)
                    # lower to startpose
                    current_pose = self.make_linear_servoj_move(current_pose, startpoint, v=0.02)
                    time.sleep(0.5)
                    #self.wait_for_steady() #does not work with servo command?

                    #start extrusion to fill defect
                    print('fill defect to pose ', self.defect_endpoints[i])
                    self.start_extrusion(40) #currently using fixed pressure for filling defects adjust for needle diameter e.g.0.42mm needle i.d. 40psi
                    current_pose = self.make_linear_servoj_move(current_pose, self.defect_endpoints[i], v = v)
                    self.stop_extrusion()
                    time.sleep(0.5)

        if retract == True:
            current_pose = self.make_movej_move(current_pose, current_pose + [0,0,0.05,0,0,0], v=0.05)
        return current_pose
 
    def get_next_layer_Z_offset(self, initial_layer_thickness, next_layer_number, offset_type = 'constant', rampscale = -0.02, min_offset = 0.5):
        """offset_type: 'constant': constant Z offset between layers, 'ramp': ramps up or down the Z offset between layers
        Note that the units are in mm! Might be reasonable to change that to meters for consistency considerations"""

        if offset_type == 'constant':
            next_layer_Z_offset = initial_layer_thickness
        elif offset_type == 'ramp':
            next_layer_Z_offset = initial_layer_thickness + initial_layer_thickness*(next_layer_number*rampscale)
            if next_layer_Z_offset < min_offset:
                next_layer_Z_offset = min_offset
        else: #unknown layer type: get some default? maybe constant?
            next_layer_Z_offset = initial_layer_thickness

        return next_layer_Z_offset

    def unclog_nozzle(self, current_pose, deposit_position, v, psi = 100):
        """moves to deposit_position and extrudes with psi=60 for 1 second before going back above current_pose"""

        self.make_movej_move(current_pose, current_pose + [0,0,0.03,0,0,0],v = v)
        self.make_movej_move(current_pose + [0,0,0.03,0,0,0], [deposit_position[0],deposit_position[1],current_pose[2] + 0.03, deposit_position[3],deposit_position[4],deposit_position[5]],v = v)
        self.make_movej_move([deposit_position[0],deposit_position[1],current_pose[2] + 0.03, deposit_position[3],deposit_position[4],deposit_position[5]], deposit_position,v = v)
        #dep_q = self.ur5e_actual_state.get_joint_angles()

        t_max = 3 #seconds
        a = time.time()
        t = 0
        while t < t_max: 
            t = time.time() - a
            self.start_extrusion(psi*t/t_max)
            self.rate.sleep()
        a = time.time()
        t = 0
        while t < t_max: 
            t = time.time() - a
            self.start_extrusion(psi-(psi*t/t_max))
            self.rate.sleep()

        self.stop_extrusion()
        self.make_movej_move(deposit_position, [deposit_position[0],deposit_position[1],current_pose[2] + 0.03, deposit_position[3],deposit_position[4],deposit_position[5]],v = v)
        self.make_movej_move([deposit_position[0],deposit_position[1],current_pose[2] + 0.03, deposit_position[3],deposit_position[4],deposit_position[5]], current_pose + [0,0,0.03,0,0,0],v = v)
        current_pose = current_pose + [0,0,0.03,0,0,0]

        return current_pose

    def get_bounding_box(self, layer_moves):
        '''returns bounding max/min x/y coordinates for model layer given as list of movement dicts in mm'''
        [max_x, max_y, min_x, min_y] = [0,0,0,0]
        for line in layer_moves:
            if line["X"] > max_x:
                max_x = line["X"]
            if line["Y"] > max_y:
                max_y = line["Y"]
            if line["X"] < min_x:
                min_x = line["X"]
            if line["Y"] < min_y:
                min_y = line["Y"]

        return [max_x, max_y, min_x, min_y]
 
    def print_layer(self, current_pose, layer_moves, height, printing_movement_speed, standard_movement_speed, ax=None):
        """works easiest when layer contains only the X- and Y movement commands i.e. using "planar_only" option in
        gcodeparser.get_first_mid_last_layer_moves() """

        last_offset = [0,0,0,0,0,0]
        for line in layer_moves:
            pdb.set_trace()
            if line['extruding'] == True:
                offset = np.array([line['X'],line['Y'], height,0,0,0])
                new_pose = self.near_touch + offset/1000 #1000: mm to m
                self.start_extrusion()
                current_pose = self.make_linear_servoj_move(current_pose, new_pose, v=printing_movement_speed, ax=ax) 
                last_offset = offset
            else:
                offset = np.array([line['X'],line['Y'], height,0,0,0])
                new_pose = self.near_touch + offset/1000 #1000: mm to m
                self.stop_extrusion()
                current_pose = self.make_movej_move(current_pose, new_pose, v=standard_movement_speed, ax=ax)
                last_offset = offset
        self.stop_extrusion() 
        #retract from current height for collision avoidance
        new_pose = current_pose+[0,0,0.005,0,0,0]
        current_pose = self.make_movej_move(current_pose, new_pose, v=standard_movement_speed, ax=ax)
        return current_pose

 

    def rot_axis_angle(self,vector1,vector2):
        # Calculate the rotation axis using the cross product
        rotation_axis = np.cross(vector1/np.linalg.norm(vector1), vector2/np.linalg.norm(vector2))
        rotation_axis /= np.linalg.norm(rotation_axis)

        # Calculate the rotation angle using the dot product and the arccosine
        rotation_angle = np.arccos(np.dot(vector1, vector2))

        # # Print the rotation vector and the rotation matrix
        # print("Rotation Vector:", rotation_angle,rotation_axis)
        return rotation_angle,rotation_axis

    def unit_axis_angle(self, a, b):
        an = sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2])
        bn = sqrt(b[0]*b[0] + b[1]*b[1] + b[2]*b[2])
        ax, ay, az = a[0]/an, a[1]/an, a[2]/an
        bx, by, bz = b[0]/bn, b[1]/bn, b[2]/bn
        nx, ny, nz = ay*bz-az*by, az*bx-ax*bz, ax*by-ay*bx
        nn = sqrt(nx*nx + ny*ny + nz*nz)
        return np.array([nx/nn, ny/nn, nz/nn]), acos(ax*bx + ay*by + az*bz)

    def extract_dictionary_from_layer_moves(self,layer_moves):
        x_list = []
        y_list = []
        extrude_list = []
        for move in layer_moves:
            x_list.append(move['X'])
            y_list.append(move['Y'])
            extrude_list.append(move['extruding'])
        xyz = np.vstack((x_list,y_list,np.zeros(len(x_list))))
        return {'X':x_list,'Y':y_list,'extruding':extrude_list, 'XYZ': xyz}

    def compute_scale_from_desired_dimensions(self,gcodeparser,max_x_dim,max_y_dim):
        first_layer_moves, mid_layer_moves_one, mid_layer_moves_two, final_layer_moves,_ = gcodeparser.get_first_mid_last_layer_moves(planar_only = True)
        model_height = final_layer_moves[0]["Z"] #mm

        self.dict_first_layer_moves = self.extract_dictionary_from_layer_moves(first_layer_moves)
        self.dict_mid_layer_moves_one = self.extract_dictionary_from_layer_moves(mid_layer_moves_one)
        self.dict_mid_layer_moves_two = self.extract_dictionary_from_layer_moves(mid_layer_moves_two)
        self.dict_final_layer_moves = self.extract_dictionary_from_layer_moves(final_layer_moves)

        length_x_list = []
        length_y_list = []

        length_x_list.append((np.max(self.dict_first_layer_moves['XYZ'],axis=1)-np.min(self.dict_first_layer_moves['XYZ'],axis=1))[0])
        length_y_list.append((np.max(self.dict_first_layer_moves['XYZ'],axis=1)-np.min(self.dict_first_layer_moves['XYZ'],axis=1))[1])
        length_x_list.append((np.max(self.dict_mid_layer_moves_one['XYZ'],axis=1)-np.min(self.dict_mid_layer_moves_one['XYZ'],axis=1))[0])
        length_y_list.append((np.max(self.dict_mid_layer_moves_one['XYZ'],axis=1)-np.min(self.dict_mid_layer_moves_one['XYZ'],axis=1))[1])
        length_x_list.append((np.max(self.dict_mid_layer_moves_two['XYZ'],axis=1)-np.min(self.dict_mid_layer_moves_two['XYZ'],axis=1))[0])
        length_y_list.append((np.max(self.dict_mid_layer_moves_two['XYZ'],axis=1)-np.min(self.dict_mid_layer_moves_two['XYZ'],axis=1))[1])
        length_x_list.append((np.max(self.dict_final_layer_moves['XYZ'],axis=1)-np.min(self.dict_final_layer_moves['XYZ'],axis=1))[0])
        length_y_list.append((np.max(self.dict_final_layer_moves['XYZ'],axis=1)-np.min(self.dict_final_layer_moves['XYZ'],axis=1))[1])
        # pdb.set_trace()
        return float(min(max_x_dim/np.max(length_x_list), max_y_dim/np.max(length_y_list)))

    
    def shift_bounds_to_zero(self,gcodeparser):
        first_layer_moves, mid_layer_moves_one, mid_layer_moves_two, final_layer_moves,_ = gcodeparser.get_first_mid_last_layer_moves(planar_only = True)
        layers = [first_layer_moves, mid_layer_moves_one, mid_layer_moves_two, final_layer_moves]
        layer_moves = []
        for layer in layers:
            layer_moves.append(self.extract_dictionary_from_layer_moves(layer))

        min_bounds = np.zeros([4,3]) 
        for i in range(4):
            # print(np.min(layer_move['XYZ'],axis=1))
            min_bounds[i,:] = np.min(layer_moves[i]['XYZ'],axis=1) 

        all_xyz = []
        for i in range(4):
            layer_moves[i]['XYZ'] -= np.min(min_bounds,axis=0).reshape(3,1) 
            all_xyz.append(layer_moves[i]['XYZ'])
        all_xyz = np.hstack(all_xyz)
        center = np.average(all_xyz,axis=1) 
        return layer_moves, center

    def print_layer_with_rotation(self, initial_pose, layer_moves, height, printing_movement_speed, standard_movement_speed):
        #layer moves are given with relative XYZ and extruding flag

        relative_positions = layer_moves['XYZ']
        relative_positions[2,:] = height

        extruding_flags = layer_moves['extruding'] 
        N_moves = len(extruding_flags)

        absolute_positions = relative_positions+initial_pose[0:3].reshape(3,1)
        roll = 180*pi/180
        pitch = -15*pi/180
        yaw = -90*pi/180 

        rv_needle = rpy2rv(roll,pitch,yaw) #
        orientations = np.tile(rv_needle.reshape(-1,1),N_moves)
        absolute_pose_list = np.vstack((absolute_positions,orientations))
        previous_pose = initial_pose
        for i in range(N_moves):
            target_pos = absolute_pose_list[:,i]
            print("Absolute target", target_pos)
            print("Relative target", relative_positions[:,i]) 
 
            extruding = extruding_flags[i] 
            if extruding:
                print('Extrude')
                if self.plot_printed_lines_enabled == True:
                    self.ax.plot([previous_pose[0],target_pos[0]],[previous_pose[1],target_pos[1]],[previous_pose[2],target_pos[2]])
                self.start_extrusion(25)
                self.move_l(target_pos, vel = printing_movement_speed, use_tool=True) 
            else:
                print('Moving')
                self.stop_extrusion() 
                self.move_l(target_pos, vel = standard_movement_speed, use_tool=True)
            print("")
            previous_pose = target_pos
            if self.plot_printed_lines_enabled == True:
                plt.pause(0.05) 
        self.stop_extrusion() 
        #retract from current height for collision avoidance
        self.move_up_from_current_pose(0.005, vel=0.05)

 
    def build_print_from_layers(self, gcodeparser, cam = False):
        """ builds a print from gcodelayers:
            Top Layer
            x * Middle Layer
            Bottom Layer
            number of middle layers is calculated from model height with respect to set layer_height

            if cam == True: after each layer: move in camera position and wait for key

            set parameters: new_layer_height, type for Z offset (constant/ramp)
        """
            #before running: 
            # make sure initial position is as close to the surface as possible, check scale: currently 20mm model
            # check movement speeds: start with 0.01 for printing
            # check target_psi: start with 30

        #Visualization in matplotlib
        self.plot_printed_lines_enabled =  False
        self.plot_printed_lines_enabled =  True
        if self.plot_printed_lines_enabled == True:
            ax = plt.axes(projection='3d') 
            self.ax = plt.axes(projection='3d') 
        else:
            ax = None

 
        self.initial_print_pose = np.array([-0.450, 0.063, 0.097-0.123, np.pi, 0, 0])
 
        self.ee_to_tooltip_delta = np.array([0.0, -0.063, 0.123])
        self.tooltip_to_ee_delta = -self.ee_to_tooltip_delta
        
        self.ee_to_tooltip_transform = np.hstack((self.ee_to_tooltip_delta,1)).reshape(4,1)
        self.tooltip_to_ee_transform = np.hstack((self.tooltip_to_ee_delta,1)).reshape(4,1)

        # get current pose note that printing will begin here with a Z offset of the first_layer_height
        # initial_pose = self.ur5e_actual_state.get_tcp_pose()
        initial_pose = self.initial_print_pose 
        current_pose =  initial_pose
        last_offset = [0,0,0,0,0,0]
        

        # initialize analog signal interface and movement speeds
        self.stop_extrusion()
        self.target_psi = 35 #35 for id0.42 #75for id 0.21 needle               #airpressure in extrusion
        printing_movement_speed = 0.006#0.010 #0.005     #movement speed in extrusion
        standard_movement_speed = 0.01 #0.05     #movement speed iF not in extrusion

        # needle inner diameter 0.21mm :     psi=70, speed = 0.005, first_layer_height= 0.2mm, layer_height=0.2mm, 
        #                                 or psi=60, speed=0.002, first layer=0.2mm, layer_height=0.165mm (for more precision on corners)
        # needle inner diameter 0.42mm: psi = 35, speed = 0.010, first_layer=0.42mm, layer_height = 0.33mm

        #re-scale model?

        #remove print position offset
        gcodeparser.remove_X_Y_offset()


        max_x_dim = 0.038
        max_y_dim = 0.038
        max_x_dim = 0.04
        max_y_dim = 0.04
        scale = self.compute_scale_from_desired_dimensions(gcodeparser,max_x_dim,max_y_dim)
        
        # scale = 1 #4
        gcodeparser.scale_model(scale)

        first_layer_moves, mid_layer_moves_one, mid_layer_moves_two, final_layer_moves,_ = gcodeparser.get_first_mid_last_layer_moves(planar_only = True)
        model_height = final_layer_moves[0]["Z"] #mm
 

        [self.dict_first_layer_moves, self.dict_mid_layer_moves_one , self.dict_mid_layer_moves_two, self.dict_final_layer_moves], self.center = self.shift_bounds_to_zero(gcodeparser)
         
        self.center[2] = 0.04
        
        self.center_pose = self.initial_print_pose.copy()
        self.center_pose[0:3] += self.center


        self.center_pose_above = self.center_pose + np.array([0,0,0.05,0,0,0])
        self.center_pose_above = self.center_pose + np.array([0,0,0.09,0,0,0])
        print("Moving Above Center Pose")
        self.move_l(self.center_pose_above, use_tool=True)

        print("Moving to Center Pose")
        self.move_l(self.center_pose, use_tool=True)
        # input()
        #set layer_height behavior 
        layercount = 0
        heighttype = 'constant' #'ramp'
        heightramp = -0.02
        first_layer_height = 0.0004 #in mm #0.4mm for 0.42 id needle#in mm #0.2mm for 0.21 id needle
        layer_height = 0.00033 #0.33 for 0.42 needle id and fine structures #in mm #0.2mm for 0.21 id needle
        layer_height = 0.0008 #0.33 for 0.42 needle id and fine structures #in mm #0.2mm for 0.21 id needle
        z_offset = first_layer_height
        next_layer_z = first_layer_height

        if heighttype == 'constant':
            heightramp = 0
 
        #declare deposit_position and initial unclog maneuver 
        deposit_position = initial_pose + np.array([0.15,0,0.003,0,0,0])   
        # current_pose = self.unclog_nozzle(current_pose, deposit_position, v = standard_movement_speed)

        unclog_after_layers = 50    # perform unclogging maneuver every x layers
        camera_after_layers = 5     # move camera in imaging position every x layers


        #print first/bottom layer on initial height
        print('First Layer ', layercount, 'z: ', next_layer_z, 'Max Height', model_height) 
        self.print_layer_with_rotation(initial_pose, self.dict_first_layer_moves, first_layer_height, printing_movement_speed,standard_movement_speed)
        layercount += 1
 

        #get height for next layer
        z_offset = self.get_next_layer_Z_offset(layer_height, layercount, heighttype ,heightramp, layer_height)
        next_layer_z += z_offset

        # self.wait_for_steady()

        #print mid layers
        while next_layer_z < model_height - layer_height:
            #print layer
            print('Layer ', layercount, 'z: ', next_layer_z)
            #alternate between the two mid_layer moves (as the infill pattern may be different)
            if layercount % 2 == 0:
                self.print_layer_with_rotation(initial_pose, self.dict_mid_layer_moves_one, next_layer_z, printing_movement_speed,standard_movement_speed)
            else:
                self.print_layer_with_rotation(initial_pose, self.dict_mid_layer_moves_two, next_layer_z, printing_movement_speed,standard_movement_speed) 
            layercount += 1
            
            #get height for next layer
            z_offset = self.get_next_layer_Z_offset(layer_height, layercount, heighttype, heightramp)
            next_layer_z += z_offset
 
            # #unclog maneuver on every 4. layer
            # if layercount % unclog_after_layers == 0: 
            #     current_pose = self.unclog_nozzle(current_pose,deposit_position, standard_movement_speed)
            #     #move camera in position and wait for key if cam == True
             

        #print final/top layer
        print('Layer ', layercount, 'z: ', next_layer_z)
        self.print_layer_with_rotation(initial_pose, self.dict_final_layer_moves, next_layer_z, printing_movement_speed,standard_movement_speed)
        layercount += 1 
  
        print('Print completed')

        if self.plot_printed_lines_enabled == True:
            self.ax.set_xlabel('X')
            self.ax.set_ylabel('y')
            self.ax.set_zlabel('z')
            plt.show()
 
    def run_python_commands_from_gcodeparser(self, gcodeparser):
        self.plot_printed_lines_enabled = True
        ax = plt.axes(projection='3d') #only for visualitzation
        # get current pose if not done before
        current_pose = self.ur5e_actual_state.get_tcp_pose()
        last_offset = [0,0,0,0,0,0]

        # initialize analog signal interface
        self.stop_extrusion()
        self.target_psi = 30
        printing_movement_speed = 0.005
        standard_movement_speed = 0.05

        x_offset = 0
        y_offset = 0
        scale = 2

        e_param = False
        #check if given gcode uses E param
        for line in gcodeparser.lines:
            if 'E' in line.params:
                e_param = True
                print('Gcode uses E parameters: only commands with set E value are going to print') #note that the E value has currently no effect on the extrusion rate
                break
        if e_param == False:
            print('gcode does not use E values: only G1 commands will print')


        ##scaling
        for line in gcodeparser.lines:
            if 'X' in line.params:
                line.update_param('X', scale*line.get_param('X'))
            if 'Y' in line.params:
                line.update_param('Y', scale*line.get_param('Y'))
            if 'Z' in line.params:
                line.update_param('Z', scale*line.get_param('Z'))

        # detect X,Y position offset to move printing start position to self.near_touch
        for line in gcodeparser.lines:
            if 'X' in line.params:
                x_offset = line.get_param('X')
            if 'Y' in line.params:
                y_offset = line.get_param('Y')
            if x_offset != 0 and y_offset != 0:
                break

        # run code line by line
        for line in gcodeparser.lines[:]:
            # print comments
            if 'retract' in line.comment or 'unretract' in line.comment:
                print('skipped retraction/unretraction')
                continue
            if line.comment != "":
                print('#' f"{line.comment}")

            # run command
            if line.command_str == 'G0':
                #check if the feedrate is changed
                #if 'F' in line.params:
                    #self.set_feedrate_to_psi(line.get_param('F)'))

                #check if the command is extruding or if its not
                if e_param == True and 'E' in line.params and self.extruding == False:
                    if 'X' in line.params or 'Y' in line.params or 'Z' in line.params:
                        if line.get_param('E') > 0:
                            self.start_extrusion()
                elif e_param ==True and 'E' not in line.params and self.extruding == True:
                    self.stop_extrusion()
                elif e_param == False and self.extruding == True:
                    self.stop_extrusion()

                #calculate new pose, send movement command 
                offset = np.array([line.get_param('X', default=last_offset[0])-x_offset, line.get_param('Y', default=last_offset[1])-y_offset, 
                                              line.get_param('Z', default=last_offset[2]), 0, 0, 0])
                new_pose = self.near_touch + offset/1000 #1000: to mm
                if self.extruding == True:
                    self.make_linear_servoj_move(current_pose, new_pose, printing_movement_speed, ax) 
                else:
                    self.make_movej_move(current_pose, new_pose, standard_movement_speed, ax) 

                #safe pose for next command
                current_pose = new_pose
                last_offset = offset

            elif line.command_str == 'G1':
                #check if the feedrate is changed
                #if 'F' in line.params:
                #    self.set_feedrate_to_psi(line.get_param('F'))

                #determine if the command is to extrude depending on E param usage in code:
                # if E params are used: only extrude if it is set in command, if unused always extrude in G1 commands
                if e_param == True and 'E' in line.params and self.extruding == False:
                    if 'X' in line.params or 'Y' in line.params or 'Z' in line.params:
                        if line.get_param('E') > 0:
                            self.start_extrusion()
                elif e_param == True and not 'E' in line.params and self.extruding == True:
                    self.stop_extrusion()
                elif e_param == False and self.extruding == False:
                    self.start_extrusion()

                #calculate new pose
                if 'X' not in line.params and 'Y' not in line.params: 
                    #only Z axis is changed
                    offset = np.array([last_offset[0], last_offset[1], line.get_param('Z', default=last_offset[2]), 0, 0, 0])
                else:
                    #movement in X-Y plane (or in all directions)
                    offset = np.array([line.get_param('X', default=last_offset[0])-x_offset, line.get_param('Y', default=last_offset[1])-y_offset, 
                                              line.get_param('Z', default=last_offset[2]), 0, 0, 0])
                new_pose = self.near_touch + offset/1000 #1000: to mm

                #send movement command and calculate inv. kinematics
                self.make_linear_servoj_move(current_pose, new_pose, printing_movement_speed, ax)

                #save pose for next command
                current_pose = new_pose
                last_offset = offset

            #elif line.command_str == 'G2': #clockwise arc to X,Y around  center point I,J, with E: amount of extrusion and F: feedrate
                #calculate length of trajectory in mm @todo: might be hard to work out
                #calculate needed time with respect to current movement speed
                #get timestamp
                #for t<= time_needed
                    # draw circle (sin(t),cos(t)...)
                    #get new t = relative time_stamp
                #t=time_needed: arc should be completed to target location: maybe check if current robot pose == given X,Y,Z?
            #elif line.command_str == 'G3': #counter-clockwise arc to X,Y around  center point I,J, with E: amount of extrusion and F: feedrate

            elif line.command_str == 'G21':
                print('units set to millimeters')  # @todo: currently no functionality, as we can only handle mm by now
            elif line.command_str in {'G28', 'G162'}: #goto home position
                print('unsupported: go to home position')
            elif line.command_str == 'G54':
                print('use coordinate system 1')  # @ToDo: will we ever need this? Still it occurs in my example Gcodes
            elif line.command_str == 'G90':
                print('position mode set to absolute positioning')  # @todo: currently no functionality, as we cannot handle relative positioning by now
            elif line.command_str in {'M02', 'M30', 'M84'}:
                break
            elif line.command_str in {'M104', 'M106', 'M107', 'M109', 'M140', 'M190'}:
                print('heating and fan commands are currently not supported, command skipped')
            else:
                print('unknown command: ' f"{line.command_str}" ', command skipped')
            self.rate.sleep()
        #end of command lines, check if extruder is turned off and got to home pos
        if self.extruding == True:
            self.stop_extrusion()
        self.make_movej_move(self.ur5e_actual_state.get_tcp_pose(),self.ur5e_actual_state.get_tcp_pose()+[0,0,0.05,0,0,0])
        print('end of printing program')
        plt.show()

    def make_strokes_vary_psi(self):
        # stroke 8: psi 20, std_printing speed
        # stroke 7: psi 30, std_printing speed
        # stroke 6: psi 40, std_printing speed
        # stroke 5: psi 50, std_printing speed
        # stroke 4: psi 60, std_printing speed
        # stroke 3: psi 70, std_printing speed
        # stroke 2: psi 80, std_printing speed
        # stroke 1: psi 90, std_printing speed

        count = 0
        i = 0
        repeat_strokes = 1

        std_printing_speed = 0.006
        std_movement_speed = 0.05
        airgap = np.array(
            [0, 0, 0.00042, 0, 0, 0])  # 0 means printing on self.near_touch z offset, try using needle inner diameter
        stroke_offset = np.array([0, -0.01, 0, 0, 0, 0])  # 1cm lateral space between strokes

        psi_list = [90, 80, 70, 60, 50, 40, 30, 20]

        init_pose = self.ur5e_actual_state.get_tcp_pose()
        camera_offset = np.array([0, +0.131, 0, 0, 0, 0])
        # camera_pose = init_pose + np.array([0.02, 0.025,0 ,0,0,0]) + camera_offset #camera should be 9cm above surface
        camera_pose = [init_pose[0], init_pose[1], 0.300, init_pose[3], init_pose[4], init_pose[5]]

        # unclog maneuver
        unclog_deposition_position = init_pose + [0, +0.15, 0.005, 0, 0, 0]
        current_pose = self.unclog_nozzle(init_pose, unclog_deposition_position, v=std_movement_speed, psi=80)

        # move above start position
        strokes_start_pose = np.array(
            [-0.6737, -0.1668, init_pose[2], init_pose[3], init_pose[4], init_pose[5]]) + airgap
        print(strokes_start_pose)
        current_pose = self.make_movej_move(current_pose, strokes_start_pose + np.array([0, 0, 0.03, 0, 0, 0]),
                                            v=std_movement_speed)

        for psi in psi_list:
            while i < repeat_strokes:
                current_pose = self.make_movej_move(current_pose, strokes_start_pose + count * stroke_offset + airgap,
                                                    v=std_movement_speed)
                self.start_extrusion(psi)
                current_pose = self.make_movej_move(current_pose, current_pose + [0.05, 0, 0, 0, 0, 0],
                                                    v=std_printing_speed)
                self.stop_extrusion()
                i += 1
            # move above stroke for camera
            mid_over_stroke = current_pose - (current_pose - strokes_start_pose) / 2 + count * stroke_offset
            camera_over_stroke = [mid_over_stroke[0], mid_over_stroke[1], camera_pose[2], mid_over_stroke[3],
                                  mid_over_stroke[4], mid_over_stroke[5]] + camera_offset
            current_pose = self.make_movej_move(current_pose, camera_over_stroke, v=std_movement_speed)
            input("Press Enter to continue...")
            count += 1
            i = 0

        # move to camera pose
        current_pose = self.make_movej_move(current_pose, camera_pose, v=std_movement_speed)
        input("Press Enter to continue...")

    def make_strokes_vary_airgap(self):
        # stroke 1: std_psi, std_printing speed, airgap 0 (minimum airgap such that needle does not hit ground)
        # stroke 2: std_psi, std_printing speed, airgap 0.21mm (0.5 needle inner diameter)
        # stroke 3: std_psi, std_printing speed, airgap 0.42mm (1 needle inner diameter)
        # stroke 4: std_psi, std_printing speed, airgap 0.84mm (2 needle inner diameter)
        # stroke 5: std_psi, std_printing speed, airgap 1.26mm (1 needle inner diameter)
        # stroke 6: std_psi, std_printing speed, airgap 1.68mm (2 needle inner diameter)
        # stroke 7: std_psi, std_printing speed, airgap 10mm (printing in air)

        count = 0
        i = 0
        repeat_strokes = 1

        std_psi = 30
        std_printing_speed = 0.006
        std_movement_speed = 0.05
        stroke_offset = np.array([0, -0.01, 0, 0, 0, 0])  # 1cm lateral space between strokes

        gap_list = [0, 0.00021, 0.00042, 0.00084, 0.00126, 0.00168, 0.01]  # m

        init_pose = self.ur5e_actual_state.get_tcp_pose()
        camera_offset = np.array([0, +0.131, 0, 0, 0, 0])
        # camera_pose = init_pose + np.array([0.02, 0.025,0 ,0,0,0]) + camera_offset #camera should be 9cm above surface
        camera_pose = [init_pose[0], init_pose[1], 0.300, init_pose[3], init_pose[4], init_pose[5]]

        # unclog maneuver
        unclog_deposition_position = init_pose + [0, +0.15, 0.005, 0, 0, 0]
        current_pose = self.unclog_nozzle(init_pose, unclog_deposition_position, v=std_movement_speed, psi=100)

        # move above start position
        strokes_start_pose = np.array([-0.6737, -0.1668, init_pose[2], init_pose[3], init_pose[4], init_pose[5]])
        current_pose = self.make_movej_move(current_pose, strokes_start_pose + [0, 0, 0.03, 0, 0, 0],
                                            v=std_movement_speed)

        for gap in gap_list:
            while i < repeat_strokes:
                current_pose = self.make_movej_move(current_pose,
                                                    strokes_start_pose + count * stroke_offset + [0, 0, gap, 0, 0, 0],
                                                    v=std_movement_speed)
                self.start_extrusion(std_psi)
                current_pose = self.make_movej_move(current_pose, current_pose + [0.05, 0, 0, 0, 0, 0],
                                                    v=std_printing_speed)
                self.stop_extrusion()
                i += 1
            # move above stroke for camera
            mid_over_stroke = current_pose - (current_pose - strokes_start_pose) / 2 + count * stroke_offset
            camera_over_stroke = [mid_over_stroke[0], mid_over_stroke[1], camera_pose[2], mid_over_stroke[3],
                                  mid_over_stroke[4], mid_over_stroke[5]] + camera_offset
            current_pose = self.make_movej_move(current_pose, camera_over_stroke, v=std_movement_speed)
            input("Press Enter to continue...")
            count += 1
            i = 0

        # move to camera pose
        current_pose = self.make_movej_move(current_pose, camera_pose, v=std_movement_speed)
        input("Press Enter to continue...")

    def make_strokes_vary_speed(self):
        # stroke 1: std_psi, 0.016
        # stroke 2: std_psi, 0.014
        # stroke 3: std_psi, 0.012
        # stroke 4: std_psi, 0.010
        # stroke 5: std_psi, 0.008
        # stroke 6: std_psi, 0.006 # 6mm/s
        # stroke 7: std_psi, 0.004

        count = 0
        i = 0
        repeat_strokes = 1

        std_psi = 40
        std_printing_speed = 0.006
        std_movement_speed = 0.05
        airgap = np.array(
            [0, 0, 0.00042, 0, 0, 0])  # 0 means printing on self.near_touch z offset, try using needle iner diameter
        stroke_offset = np.array([0, -0.01, 0, 0, 0, 0])  # 1cm lateral space between strokes

        speed_list = [0.016, 0.014, 0.012, 0.010, 0.008, 0.006, 0.004]

        init_pose = self.ur5e_actual_state.get_tcp_pose()
        camera_offset = np.array([0, +0.131, 0, 0, 0, 0])
        # camera_pose = init_pose + np.array([0.02, 0.025,0 ,0,0,0]) + camera_offset #camera should be 9cm above surface
        camera_pose = [init_pose[0], init_pose[1], 0.300, init_pose[3], init_pose[4], init_pose[5]]

        # unclog maneuver
        unclog_deposition_position = init_pose + [0, +0.15, 0.005, 0, 0, 0]
        current_pose = self.unclog_nozzle(init_pose, unclog_deposition_position, v=std_movement_speed, psi=100)

        # move above start position
        strokes_start_pose = np.array(
            [-0.6737, -0.1668, init_pose[2], init_pose[3], init_pose[4], init_pose[5]]) + airgap
        current_pose = self.make_movej_move(current_pose, strokes_start_pose + [0, 0, 0.03, 0, 0, 0],
                                            v=std_movement_speed)

        for speed in speed_list:
            while i < repeat_strokes:
                current_pose = self.make_movej_move(current_pose, strokes_start_pose + count * stroke_offset + airgap,
                                                    v=std_movement_speed)
                self.start_extrusion(std_psi)
                current_pose = self.make_movej_move(current_pose, current_pose + [0.05, 0, 0, 0, 0, 0], v=speed)
                self.stop_extrusion()
                i += 1
            # move above stroke for camera
            mid_over_stroke = current_pose - (current_pose - strokes_start_pose) / 2 + count * stroke_offset
            camera_over_stroke = [mid_over_stroke[0], mid_over_stroke[1], camera_pose[2], mid_over_stroke[3],
                                  mid_over_stroke[4], mid_over_stroke[5]] + camera_offset
            current_pose = self.make_movej_move(current_pose, camera_over_stroke, v=std_movement_speed)
            input("Press Enter to continue...")
            count += 1
            i = 0

        # move to camera pose
        current_pose = self.make_movej_move(current_pose, camera_pose, v=std_movement_speed)
        input("Press Enter to continue...")

    def log_joint_angles_in_movel(self):
        current_pose = self.ur5e_actual_state.get_tcp_pose()
        target_pose = current_pose + [0.1, 0, 0, 0, 0, 0]
        v = 0.02
        total_time = self.calc_move_duration(current_pose, target_pose, v)

        with open(os.path.dirname(os.path.abspath(__file__))+'/linear_movement_q.csv', 'w', newline='') as f:
            # create csv writer
            writer = csv.writer(f)
            writer.writerow([self.ur5e_actual_state.get_joint_angles(),0]) #write init q
            t = 0

            self.send_input_to_ur5e("movel",target_pose, 0.5, v, 0, 0) 
            starttime = time.time()
            while t < total_time:
                t += 0.008
                writer.writerow([self.ur5e_actual_state.get_joint_angles(),t])
                #log t
                time.sleep(0.008 - (time.time() - (t + starttime)))
            self.wait_for_steady()

    def mass_flowrate_measurement(self):
        """ Made for static deposition of material on the weight scale. Set unclog_position beforehand and make sure
        the system is not colliding with the scale while moving between unclog and deposition position.
        The 'Time', 'Weight','XYZ', ' PSI', "Movement Speed [mm/s]", and "Needle Inner Diameter [mm]" are logged to a csv file. """
        #preparation:   make sure robot is in position (self.near_touch above scale)
        #               make sure unclogging maneuver position is not above scale
        #               check needle inner diameter
        # todo:         evaluate if airgap and extrusion time are reasonable choices for final measurement
        #               workout pressure range

        #start at printing position
        init_pose = self.ur5e_actual_state.get_tcp_pose()
        deposit_pose = np.array([-0.4182,  0.0145,  0.0945,  3.14155, -0.00005, -0.00004])

        #params
        psi = 80
        unclog_psi = 90
        airgap = 0.03 #in m; 0 means printing on init_pose height
        extrusion_time = 120 #seconds
        needle_id = 0.26 #mm 0.21

        target_pose = init_pose + [0, 0, airgap, 0, 0, 0]
        csvline=[] #t, weight, xyz, psi, movement_speed

        while self.scale_data_received == False:
            print('scale not connected')
            time.sleep(1)

        #unclog maneuver
        self.unclog_nozzle(init_pose, deposit_pose, 0.1, psi = unclog_psi)

        # move to target pose
        new_q = get_ik_solution_near_q(target_pose, self.ur5e_actual_state.get_joint_angles())
        self.send_input_to_ur5e("movej",new_q, 0.2, 0.05, 0, 0)
        self.wait_for_steady()

        with open(os.path.dirname(os.path.abspath(__file__))+'/scale_values.csv', 'w', newline='') as f:
            # create csv writer
            writer = csv.writer(f)
            writer.writerow(['Time', 'Weight','XYZ', ' PSI', "Movement Speed [mm/s]", "Needle Inner Diameter [mm]"])

            #extrude for extrusion_time seconds while weighting
            print('start extrusion and weighting for ', extrusion_time, ' seconds')
            timestamp = time.time()
            t = time.time() - timestamp
            self.start_extrusion(psi)
            # self.send_input_to_ur5e("analog", psi)
            while t < extrusion_time:
                writer.writerow([t,self.scale_value,target_pose[0:3],psi,0, needle_id])
                self.rate.sleep()
                t = time.time() - timestamp

            self.stop_extrusion()
            # self.send_input_to_ur5e("analog", [0])

            new_q = get_ik_solution_near_q(target_pose + [0,0,0.05,0,0,0], self.ur5e_actual_state.get_joint_angles())
            self.send_input_to_ur5e("movej",new_q, 0.2, 0.05, 0, 0)

            #read scale for 5 more seconds after extrusion should have ended
            while t<extrusion_time+5:
                writer.writerow([t,self.scale_value,target_pose[0:3],0,0])
                self.rate.sleep()
                t = time.time() - timestamp

            self.wait_for_steady()

    def run_main_loop(self):
        """ Loop for the execution of commands. This is where the current usage scenario is determined/choosen.
        make sure to set self.near_touch such that the needle is positioned without any noticable air gap.
        The needle should be as close to the surface as possible without making contact only having a small security gap
        to cover repeatability and movement accuracy of the robot.  """

        print("Starting controller loop") 
        while not self.data_received:
            # print("Waiting for Data")
            time.sleep(0.01)  
        self.move_to_initial_pose(0.1) 

        """Find a good near_touch position by moving the robot manually. Some previously 
        used positions for orientation are given here:
        
        #self.tip_near_touch = np.array([-0.5814,  0.0145,  0.1735, 3.14155, -0.00005, -0.00004]) #[-0.602, 0.482, 0.1031, 1.760, -1.760, -0.729])#strokes: [-0.6314,  0.0145,  0.0935,  3.14155, -0.00005, -0.00004] scale: [-0.6735, 0.0156, 0.1733, 3.14155, -0.00005, -0.00004] petridish:[-0.6314,  0.0145,  0.0935,  3.14155, -0.00005, -0.00004]
        #tcp_near_touch = transformMatrix_from_pose(np.hstack((self.tip_near_touch[0:3], 3.14155, -0.00005, -0.00004)))@self.needletip_to_ee_transform
        #self.near_touch = np.array([tcp_near_touch[0][0],tcp_near_touch[1][0],tcp_near_touch[2][0], 3.14155, -0.00005, -0.00004])
        """

        self.near_touch = np.array([-0.6014, 0.0185, 0.1095, 0.0, 3.034, 0.813])
 

        """Commands to be executed following on from the near_touch position goes here. You can find some example usage
        of commands below (commented). Technically everything from here to the "else" statement can be deleted and
        desired commands inserted"""

        #bioprint from gcode:
        # open gcode file and store contents as variable
        gcodename = '5-5-5hollow_cube'
        gcodename = '5-5-5cube_alternatingInfill'
        with open(os.path.dirname(os.path.abspath(__file__))+'/example_gcodes/'+gcodename+'.gcode', 'r') as f: #5-5-5hollow_cube #5-5-5_cube infill_20lines #5-5-5cube_alternatingInfill # radiu_8_hollow_hexagon
            gcode = f.read()
        gcodeparser = GcodeParser(gcode)  # get parsed gcode lines
        self.build_print_from_layers(gcodeparser, cam = False) 

        #self.make_strokes_vary_psi()
        #self.make_strokes_vary_speed()    
        #self.make_strokes_vary_airgap()      



if __name__ == "__main__":
    controller_node()
 
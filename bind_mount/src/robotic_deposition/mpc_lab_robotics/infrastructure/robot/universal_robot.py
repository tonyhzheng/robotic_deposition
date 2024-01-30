#!/usr/bin/env python3 
"""
Class for initializing the connections to the UR5e and sending/receiving data
Adapted from https://github.com/UniversalRobots/RTDE_Python_Client_Library
Author: Tony Zheng
"""

import time
import logging 
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config 

import socket
import sys
import numpy as np
import signal
import pdb 
from mpc_lab_robotics.utils.utils import *
import os
import threading
import pdb

class UR_Robot():
    def __init__(self, sim = True, load_program_and_play = True, model = 'ur5e', config_file = 'rtde_recipe_file.xml', program_name = 'rtde_control_loop.urp', frequency = 500 ):
        
        self.sim = sim
        self.load_program_and_play = load_program_and_play
        if self.sim: 
            self.robot_IP_address = "localhost" # Simulation IP address
            # self.robot_IP_address = "192.168.56.101" # Simulation IP address
        else:
            self.robot_IP_address = "192.168.1.2" # Real ur5 IP address
            self.robot_IP_address = "192.168.50.5" # Real ur5 IP address 
        self.model = model
        
        file_path = os.path.dirname(os.path.abspath(__file__))+'/ur5e_control_box_assets/'
        self.config_file = file_path + config_file
        self.program_name = program_name
        self.frequency = frequency

        self.inputDict = {}
        self.outputDict = {}
        self.inputKeys = {}

        self.rtde_port = 30004
        self.socket_port = 29999
        self.timeout = 5

        self.initialize() 

        print(self.robot_IP_address )
        print(model + " controller loaded")

    def initialize(self):
        signal.signal(signal.SIGINT, self.stop_robot)  
        logging.getLogger().setLevel(logging.ERROR)
        self.initialize_controllers()
        self.initialize_rtde()
        self.initialize_socket()

        if self.load_program_and_play:
            self.unlock_protective_stop()
            self.load_Polyscope_program(self.program_name) 
            print("Program loaded: " + self.program_name)

            print(self.play_Polyscope_program())
            self.send_command(np.zeros(self.num_inputs))
            # time.sleep(0.1)

        # watchdog keeps the program active while this script is running (to prevent robot from continuing movements after error)
        watchdog_thread = threading.Thread(target=self.watchdog_task, name='watchdog') 
        watchdog_thread.setDaemon(True)
        watchdog_thread.start()
        self.previous_time = time.time()


    def initialize_controllers(self):

        self.loop_rate = 125
        self.dt        = 1.0/self.loop_rate

        # Joint position control params
        self.velocity_qpos     = 0 # not used in urscript version
        self.acceleration_qpos = 0 # not used in urscript
        self.blocking_time     = self.dt # command to robot is sent for this duration, other calls blocked 
        self.lookahead_time    = 0.1 # smoothing lookahead time. range =(0.03,0.2)
        self.gain              = 300 # proportional gain, range = (100,2000)

        # Joint velocity control params
        self.speed_scale       = 10
        self.acceleration_qvel = 1.5 # rad/s^2 

        #End effector position control params

        self.velocity_eepos     = 0.3 
        self.acceleration_eepos = 1  
        self.blend              = 0

        self.acceleration_tcp_vel            = 0.5 #m.s^2
        self.acceleration_rotational_tcp_vel = 0.5 # rad/s^2 

        
        # Force control params
        """: Remember to tare the force-torque sensor by calling
            zero_ftsensor() first. Avoid movements parallel to compliant axes and
            high deceleration (consider inserting a short sleep command of at
            least 0.02s) just before entering force mode. Avoid high acceleration in
            force mode as this decreases the force control accuracy"""
        self.task_frame        = [0, 0, 0, 0, 0, 0] 
        self.force_type        = 2
        self.compliance_vector = [1, 1, 1, 0, 0, 0] 
        self.limits            = [2, 2, 2, 0.01, 0.01, 0.01] 


        self.acceleration_stop = 2
        self.analog_psi_value = 0

    def watchdog_task(self, kick_freq = 5):
        i = 0
        rem = int(125/kick_freq)
        while True:
            try:
                self.rtde_update_state()
                if i % rem == 0:
                    self.rtde_kick_watchdog()
            except Exception as e:
                # print(e)
                pass
            i += 1
            time.sleep(1.0/125)

    def initialize_rtde(self): 
        """From Universal Robot remote operation connector.py """
        self.con = rtde.RTDE(self.robot_IP_address, self.rtde_port)
        self.con.connect()
        print("RTDE Connected")
        self.controlVersion = self.con.get_controller_version()
        if not self.controlVersion[0] == 5:
            print("Robot connected is not an E-series. Exiting...")
            sys.exit()
        if self.controlVersion[1] < 8:
            print("Current Polyscope software is below 5.8. Some commands may be missing.")
            # sys.exit()
        self.rtde_initialize_recipes()
            
        if not self.con.send_start():
            sys.exit()

        self.programState = {
            0: 'Stopping',
            1: 'Stopped',
            2: 'Playing',
            3: 'Pausing',
            4: 'Paused',
            5: 'Resuming',
            6: 'Retracting'
        }

        self.robotOperationalState = {
            0: ' ',
            1: 'Normal',
            2: ' ',
            3: 'Playing',
            4: ' ',
            5: 'Freedrive',
            6: ' '
        }

        self.command_type = {
            'joint_pos'             : 1,
            'joint_vel'             : 2,
            'joint_pos_long_travel' : 3,
            'tcp_pos_long_travel'   : 4,
            'tcp_vel'               : 5,
            'force'                 : 6,
            'stop'                  : 7,
            'tcp_pos'               : 8,

            'servoj'                : 1,
            'speedj'                : 2,
            'movej'                 : 3,
            'movel'                 : 4,
            'speedl'                : 5,
            'servol'                : 8,
            'reset_force_sensor'    : 9,

            'analog_value_update'   : 98,
            'analog'                : 99,
        }

        self.receiving = False
        self.in_long_movement = False 
        self.state = self.rtde_update_state() 
        for i in range(100):
            # print(self.rtde_update_state())
            self.rtde_update_state()
            time.sleep(0.001)
        # print(self.state.actual_TCP_pose)
        # print(dir(self.initial_state))  

    def rtde_update_state(self):
        self.state = self.con.receive()
        return self.state

    def get_state(self):
        # self.state = self.con.receive()
        # return self.state  
        return self.state
        
    def get_actual_tcp_pose(self): 
        return np.array(self.state.actual_TCP_pose)
        
    def get_actual_tcp_speed(self):
        return np.array(self.state.actual_TCP_speed)

    def get_actual_tcp_force(self):
        if self.sim:
            z_threshold = 0.14 #table at 0.2m in base frame
            if self.state.target_TCP_pose[2]>z_threshold:
                return np.array(self.state.actual_TCP_force)
            else:
                k = 2/0.01 #2 Newtons at 1cm below the threshold
                # noise = np.clip(-1,1,np.random.rand())*1.0
                # return np.array([0,0,(z_threshold-self.state.target_TCP_pose[2])*k+noise,0,0,0])
                return np.array([0,0,(z_threshold-self.state.target_TCP_pose[2])*k ,0,0,0])
        else:
            return np.array(self.state.actual_TCP_force)

    def get_actual_joint_angles(self):
        return np.array(self.state.actual_q)

    def get_actual_joint_velocities(self):
        return np.array(self.state.actual_qd)

    def get_target_tcp_pose(self): 
        return np.array(self.state.target_TCP_pose)
        
    def get_target_tcp_speed(self):
        return np.array(self.state.target_TCP_speed)

    def get_target_tcp_force(self):
        return np.array(self.state.target_TCP_force)

    def get_target_joint_angles(self):
        return np.array(self.state.target_q)

    def get_target_joint_velocities(self):
        return np.array(self.state.target_qd)

    def get_robot_is_steady(self):
        return self.state.output_bit_register_64

    def get_program_status(self):
        return self.programState.get(self.state.runtime_state)

    def get_joint_torques(self):
        return np.array([self.state.output_double_register_0, self.state.output_double_register_1, self.state.output_double_register_2, self.state.output_double_register_3, self.state.output_double_register_4, self.state.output_double_register_5])

    def get_robot_operational_status(self):
        # print("Safety Status ", self.state.safety_status)
        # print("Safety Status Bits ", self.state.safety_status_bits)
        # # print(self.state.safety_mode) # Deprecated
        # print("Robot Mode ", self.state.robot_mode)
        # print("Robot Status Bits ", self.state.robot_status_bits)
        return self.robotOperationalState.get(self.state.robot_status_bits)

    def rate_sleep(self, rate = 125):
        elapsed_time = time.time()-self.previous_time
        if elapsed_time<(1/rate):
            time.sleep(max(1/rate-elapsed_time, 0.0001))
        self.previous_time = time.time()

    def robot_idle(self, total_time , kick_freq = 10):
        start_time = time.time()
        while (time.time()-start_time)< total_time:
            self.rtde_kick_watchdog()
            self.rate_sleep(kick_freq) 

    def rtde_kick_watchdog(self):
        # self.rtde_send("watchdog", "input_int_register_0", 0)
        self.con.send(self.watchdog)

    def rtde_initialize_recipes(self):
        # Try to add all additional recipe keys to setup.
        conf = rtde_config.ConfigFile(self.config_file)
        try:
            # Setup the "state" outputs which are core to this example.
            self.state_names, self.state_types = conf.get_recipe('state') 
            self.con.send_output_setup(self.state_names, self.state_types, frequency=self.frequency)

            self.command_inputs_names, self.command_inputs_types = conf.get_recipe('command_inputs')
            self.command_inputs = self.con.send_input_setup(self.command_inputs_names, self.command_inputs_types)  
            self.command_inputs = self.initialize_input(self.command_inputs, self.command_inputs_names) 

            self.watchdog_names, self.watchdog_types = conf.get_recipe('watchdog')
            self.watchdog = self.con.send_input_setup(self.watchdog_names, self.watchdog_types) 
            self.watchdog.input_int_register_0 = 0

            self.setp_names, self.setp_types = conf.get_recipe('setp')
            self.setp_inputs = self.con.send_input_setup(self.setp_names, self.setp_types)  
            self.inputDict['setp'] = self.initialize_input(self.setp_inputs, self.setp_names)  
            self.inputKeys['setp'] = self.setp_names

            self.num_inputs = len(self.command_inputs_names)
            self.command_inputs = self.list_to_setp(self.command_inputs, np.zeros(self.num_inputs)) 
            
        # Example error handling if a recipe key wasn't found.
        except KeyError as e:
            logging.debug(f'An item was not found in the XML: {e}')

    def initialize_input(self, input_setup, names):
        for item in names:
            input_setup.__dict__[item] = 0
        return input_setup


    def list_to_setp(self, setp, inputs):  
        for i in range (0,min(len(inputs),self.num_inputs)):
            setp.__dict__["input_double_register_%i" % i] = inputs[i]
        return setp

    def setp_to_list(self, setp):
        tolist = [] 
        for i in range(0,self.num_inputs):
            tolist.append(setp.__dict__["input_double_register_%i" % i])
        return tolist

    def send_command_type(self, command_type, inputs):
        cmd = self.command_type.get(command_type)
        if cmd == 1:
            self.send_joint_pos_command(inputs)
        elif cmd == 2:
            self.send_joint_vel_command(inputs)
        elif cmd == 3:
            self.send_joint_pos_long_travel_command(inputs)
        elif cmd == 4:
            self.send_tcp_pos_long_travel_command(inputs)
        elif cmd == 5:
            self.send_tcp_vel_command(inputs)
        elif cmd == 6:
            self.send_force_command(inputs)
        elif cmd == 7:
            self.send_stop_command()
        elif cmd == 8: 
            self.send_tcp_pos_command(inputs)
        elif cmd == 9: 
            self.send_reset_force_sensor_command(inputs)
        elif cmd == 98:
            self.update_analog_value(inputs)
        elif cmd == 99:
            self.send_analog_only_command(inputs)
        else:
            print("Invalid command type")


    def send_reset_force_sensor_command(self,inputs): 
        self.send_command(self.command_type.get("reset_force_sensor"), np.hstack((inputs, np.zeros(self.num_inputs-2))))

    def send_joint_pos_command(self,joint_q): 
        self.send_command(self.command_type.get("joint_pos"), joint_q, self.velocity_qpos, self.acceleration_qpos, self.blocking_time, self.lookahead_time, self.gain)

    def send_joint_vel_command(self,joint_vel): 
        self.send_command(self.command_type.get("joint_vel"), joint_vel, self.acceleration_qvel, self.blocking_time)

    def send_joint_pos_long_travel_command(self, joint_q, joint_acc_com = 1.4, joint_vel_com=1.05, time_ignore_a_v = 0): 
        # print(f"Starting long joint position command to {joint_q[0:6]}") 
        self.send_command(self.command_type.get("joint_pos_long_travel"), joint_q, joint_acc_com, joint_vel_com, time_ignore_a_v,  self.blend)
        self.in_long_movement = True
        time.sleep(0.1)  
        while not self.get_robot_is_steady(): 
            # self.rtde_kick_watchdog() 
            time.sleep(1/self.frequency) 
        
        self.in_long_movement = False
        # print("Long joint position command completed")
        logging.info("Long joint position command completed")

    def send_tcp_pos_long_travel_command(self, pose, tcp_accel = 1.2, tcp_vel= 0.25, time_ignore_a_v = 0):  
        # print(f"Starting long tcp position command to {pose[0:6]}")
        self.send_command(self.command_type.get("tcp_pos_long_travel"), pose, tcp_accel, tcp_vel, time_ignore_a_v,  self.blend)
        self.in_long_movement = True
        time.sleep(0.1)  
        while not self.get_robot_is_steady(): 
            # self.rtde_kick_watchdog() 
            time.sleep(1/self.frequency) 
        self.in_long_movement = False
        logging.info("Long tcp position command completed")
 

    def send_tcp_pos_command(self, inputs):  
        self.send_command(self.command_type.get("tcp_pos"), inputs, self.velocity_qpos, self.acceleration_qpos, self.blocking_time, self.lookahead_time, self.gain)

    def send_tcp_vel_command(self, tcp_vel):  
        self.send_command(self.command_type.get("tcp_vel"), tcp_vel, self.acceleration_tcp_vel, self.blocking_time*0,  self.acceleration_rotational_tcp_vel)

    def send_force_command(self,wrench): 
        self.send_command(self.command_type.get("force"), wrench, self.compliance_vector, self.limits)

    def send_stop_command(self): 
        self.send_command(self.command_type.get("stop"), self.acceleration_stop, np.zeros(self.num_inputs-2))
 

    def send_analog_only_command(self, analog_out_value): 
        self.analog_psi_value = np.clip(analog_out_value,0,1)
        self.send_command(self.command_type.get("analog"), np.hstack((np.zeros(18),self.analog_psi_value)))

    def update_analog_value(self,analog_out_value):
        self.analog_psi_value = np.clip(analog_out_value,0,1)

    def send_command(self, *inputs): 
        try: 
            if not self.in_long_movement: 
                # print(np.hstack(inputs))
                inputs_to_send = np.hstack(inputs) 
                self.command_inputs = self.list_to_setp(self.command_inputs, inputs_to_send) 
                self.command_inputs .__dict__["input_double_register_19"] = self.analog_psi_value
                self.con.send(self.command_inputs)
        except:
            print("Error sending command")
            pdb.set_trace()

    def rtde_send(self, key, field, value):
        """
        Send RTDE inputs to the robot with a given list of fields and values.
        All fields within a key must be sent with an associated value.
        :param str key: the key to pull the corresponding inputs from.
        :param field: A list of fields (RTDE inputs) to send updated values to. Can also be a string value when
        specifying a single field.
        :param value: A list of updated input values to send to RTDE. Must match the order and type of
        the field parameter. Can be a single value in the case of sending a single field.
        """
        if type(field) is not list:
            self.inputDict[key].__dict__[field] = value
        else:
            for i in range(len(field)):
                self.inputDict[key].__dict__[field[i]] = value[i]
        self.con.send(self.inputDict[key]) 

    def rtde_sendall(self, key, value):
        """
        Send RTDE inputs to the robot with a given list of values. The order of values matches the recipe XML file
        for a given key.
        :param str key: The key to pull the corresponding inputs from.
        :param value: A list of updated input values to send to RTDE.
        """
        if type(value) is not list:
            self.inputDict[key].__dict__[self.inputKeys[key][0]] = value
        else:
            for i in range(len(value)):
                self.inputDict[key].__dict__[self.inputKeys[key][i]] = value[i]
        self.con.send(self.inputDict[key])

    def rtde_shutdown(self):
        """
        Safely disconnects from RTDE and shuts down.
        :return: None
        """ 
        self.send_command(np.zeros(self.num_inputs))
        time.sleep(0.1)
        # self.con.send_pause()
        self.con.disconnect()
        time.sleep(0.1)


    """ For sending system level commands via socket"""

    def initialize_socket(self):
        """From Universal Robot remote operation Dashboard.py """
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket_connect()
        print("Socket Connected")
        # Check to see if robot is in remote mode.
        remoteCheck = self.socket_sendAndReceive('is in remote control')
        if 'false' in remoteCheck:
            logging.warning('Robot is in local mode. Some commands may not function.')
            if not self.sim and self.load_program_and_play:
                print('Error: Change Polyscope to remote mode')
                sys.exit()

    def socket_connect(self):
        self.sock.settimeout(self.timeout)
        self.sock.connect((self.robot_IP_address, self.socket_port))
        # Receive initial "Connected" Header
        self.sock.recv(1096)

    def socket_sendAndReceive(self, command):
        try:
            self.sock.sendall((command + '\n').encode()) 
            return self.socket_get_reply()
        except (ConnectionResetError, ConnectionAbortedError):
            logging.warning('The connection was lost to the robot. Please connect and try running again.')
            self.socket_close()
            sys.exit()

    def socket_get_reply(self):
        """
        read one line from the socket
        :return: text until new line
        """
        collected = b''
        while True:
            part = self.sock.recv(1)
            if part != b"\n":
                collected += part
            elif part == b"\n":
                break
        return collected.decode("utf-8")

    def socket_shutdown(self):
        # self.stop_Polyscope_program()
        try:
            self.sock.sendall(('stop'+ '\n').encode()) 
        except:
            print("Error stop")
        time.sleep(0.1)
        self.socket_close()
        time.sleep(0.1)

    def socket_close(self):
        return self.sock.close()

    def stop_Polyscope_program(self):
        # time.sleep(0.01)
        return self.socket_sendAndReceive('stop')

    def play_Polyscope_program(self): 
        return self.socket_sendAndReceive('play')

    def unlock_protective_stop(self):
        reset = "Cleared"
        while self.socket_get_safetystatus()=='Safetystatus: PROTECTIVE_STOP':
            print("Attempting protective stop unlock (Min 5s)")
            reset = self.socket_sendAndReceive('unlock protective stop')
            time.sleep(1)
        return reset

    def load_Polyscope_program(self, program_name):
        return self.socket_sendAndReceive('load '+ program_name)
    
    def socket_get_robotmode(self):
        return self.socket_sendAndReceive('robotmode')

    def socket_get_safetystatus(self):
        return self.socket_sendAndReceive('safetystatus')

    def socket_get_programState(self):
        return self.socket_sendAndReceive('programState')

    def stop_robot(self, *args, **kwargs):
        self.rtde_shutdown()
        self.socket_shutdown()
        print("RTDE and Socket closed")
        time.sleep(0.5)
        # os._exit(0)
        sys.exit()
        # print("Exited")
 
if __name__ == "__main__":
    sim = True
    # sim = False
    load_program_and_play = False
    # load_program_and_play = True
    ur5e = UR_Robot(sim=sim, load_program_and_play=load_program_and_play, program_name='rtde_control_loop.urp')  
 
    start_q = ur5e.get_actual_joint_angles()
    start_pos = ur5e.get_actual_tcp_pose()
    t_start = time.time()
    for i in range(5000): 
        a = time.time()
        t = time.time()-t_start
        state = ur5e.get_state()
        if state is not None:
            # ur5e.rtde_kick_watchdog()
            print("--------------------------------------",i)
            print(repr(ur5e.get_actual_joint_angles()))
            print(repr(ur5e.get_actual_tcp_pose()))
            # q_now = ur5e.get_actual_joint_angles()
            # pos_des = start_pos + np.array([sin(t)*0.01,cos(t)*0.01,0,sin(t)*0.01,0,0])
            # ur5e.send_tcp_pos_command(np.hstack(([pos_des, q_now])))
            # print(ur5e.get_program_status())
            # print(ur5e.get_robot_operational_status())
            # print(ur5e.socket_get_programState())
            # print(ur5e.socket_get_robotmode())
            # print(ur5e.socket_get_safetystatus())
            # print(ur5e.get_robot_is_steady()) 
        ur5e.rate_sleep(125)
        print(time.time()-a, 1/(time.time()-a))

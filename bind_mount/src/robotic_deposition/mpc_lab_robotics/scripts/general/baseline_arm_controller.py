#!/usr/bin/env python3
"""
Class for controlling the UR5e with some basic movements
Author: Tony Zheng
"""
import numpy as np
from numpy.linalg import norm,inv,det, matrix_power
from numpy import sign, pi ,sin 
import time 
 
from mpc_lab_robotics.infrastructure.sensors.camera import camera, VideoStreamWidget, VideoCaptureBufferless
from mpc_lab_robotics.infrastructure.robot.ur_rtde_state import ur_rtde_state
from mpc_lab_robotics.utils.utils import R_XYZ, rpy2rv, transformMatrix_from_pose, interpolate_eulers, rv2rpy, interpolate_rvs

from robotic_deposition.msg import ur5e_data, ur5e_control

import sys
import cv2
import os
import rospy
from std_msgs.msg import Bool
import pandas as pd
import datetime
from collections import deque
import signal
import pdb

np.set_printoptions(suppress=True,precision=5,linewidth = np.inf)

class baseline_arm_controller():
    def __init__(self, frequency = 125, use_camera = False): 

        self.frequency = frequency
        self.use_camera = use_camera
        
        self.initialize_ros() 
        self.define_initial_params()
        self.initialize_additional_functions()
 
        signal.signal(signal.SIGINT, self.stop_controller)   

        time.sleep(1)
        self.run_main_loop()

    def initialize_additional_functions(self):
        pass

    def stop_controller(self, *args, **kwargs):
        print("Stopping controller")
        sys.exit(0) 

    def initialize_ros(self):
        rospy.init_node('controller_node', anonymous=False)
        self.loop_rate = 1/self.frequency
        self.rate =  rospy.Rate(self.frequency)
        
        self.ur5e_actual_state = ur_rtde_state()
        rospy.Subscriber('/ur5e/rtde_sensor_data/actual', ur5e_data,  self.callback_actual_ur5e_data, queue_size = 10)

        self.low_level_control_input_publisher = rospy.Publisher('/controllers/ur5e_low_level_control', ur5e_control, queue_size = 10)
        self.task_completed_publisher = rospy.Publisher('/finished_task', Bool, queue_size = 10)

        self.low_level_control_input = ur5e_control()
        self.data_received = False
        print("control_node ROS initialized")

    def define_initial_params(self): 
        if self.use_camera:
            self.camera = camera()
        self.cell_number = 1   
 

        self.t_check = time.time()

        self.n_touch_x = 3
        self.n_touch_y = 5
        self.x_touch = np.linspace(-0.715,-0.375-0.24,self.n_touch_x) 
        self.y_touch = np.linspace(-0.240, 0.260,self.n_touch_y) 

        rv = rpy2rv(pi,0,-pi/2) 
        ymesh,xmesh = np.meshgrid(self.y_touch,self.x_touch)
        self.cell_pos = [(xmesh.flatten()[i],ymesh.flatten()[i]) for i in range(self.n_touch_x*self.n_touch_y )]
        print(self.cell_pos)
        
        self.cell_home = np.hstack((self.x_touch[0],self.y_touch[0],0.148,rv))
        self.cell_home = np.hstack((self.x_touch[0],self.y_touch[0],0.192,rv))


        #orientation for pencil pointed into the surface perpendicular
        roll = 180*pi/180
        pitch = 0*pi/180
        yaw = -90*pi/180 

        self.rv_default = rpy2rv(roll,pitch,yaw) #
        self.rotation_matrix_default = R_XYZ(roll,pitch,yaw) 
        self.tool_length = 0.1
        self.ee_to_tooltip_transform = np.array([0, 0.0,self.tool_length,1]).reshape(4,1)
        self.tooltip_to_ee_transform = np.array([0, 0.0, -self.tool_length,1]).reshape(4,1)

        self.height_ee_indicator = 0.0063 #length of the indicator knob on end effector
            # self.z_ee_touch_table = 0.0104 #with slab on top of welding cart
        self.z_ee_knob_touch_table = -0.0242 #height in base frame that the indicator knob touches the weldingcart
        self.z_ee_touch_table = self.z_ee_knob_touch_table -self.height_ee_indicator #height that the ee touches table
        self.z_ee_touch_table = -0.0305

        #float glass from ace hardware 2.5mm thickness

    def move_to_initial_pose(self,speed=0.05): 
        print("Moving to initial position") 

        self.pose_home = np.array([-0.41188,  0.13319,  0.41316, -2.22148,  2.2214 , -0.00018]) 
        self.joint_q_home = np.array([-3.14201, -1.39681, -1.91919, -1.39823,  1.57251,  3.14203])

        self.stay_at_initial = False 

        total_time = self.calc_move_duration(self.pose_home, self.ur5e_actual_state.get_tcp_pose(), speed)
        self.send_input_to_ur5e("movej",self.joint_q_home , 0.5, 0.5, total_time, 0)
        self.wait_for_steady()

    def move_j(self,q,total_time=5):
        print("Movej")
        print("Start", self.ur5e_actual_state.get_joint_angles())
        print("Target", q)
        self.send_input_to_ur5e("movej",q , 0.5, 0.5, total_time, 0)
        self.wait_for_steady()
   
    def callback_actual_ur5e_data(self,msg):   
        self.ur5e_actual_state.add_data_from_msg(msg)
        self.data_received = True
 
    def calc_move_duration(self, pose_1, pose_2, speed):
        distance = norm(pose_1[0:3]-np.array(pose_2[0:3])) 
        # print(pose_1[0:3]-np.array(pose_2[0:3]),distance)
        total_time = distance/speed
        return total_time
    
    def move_up_from_current_pose(self, vertical_distance = 0.05, vel= 0.2, use_tool = False):
        new_pose = self.ur5e_actual_state.get_tcp_pose()+[0,0,vertical_distance,0,0,0]  
        print(f"Lifting to pose: {new_pose}")
        self.move_l(new_pose, vel= vel, use_tool=use_tool)


    def wait_for_steady(self):
        # URscript is_steady() function waits for 0.5s of 0 vel
        time.sleep(0.1)
        while not self.ur5e_actual_state.get_robot_is_steady():
            # print("Steady",self.ur5e_actual_state.get_robot_is_steady(),self.ur5e_actual_state.get_timestamp())
            time.sleep(0.001)
        # time.sleep(0.2)
        print(f"Steady at pose: {self.ur5e_actual_state.get_tcp_pose()}")

    def send_force_sensor_reset(self): 
        print("RESETTING FORCE SENSOR")
        for i in range(5):
            self.send_input_to_ur5e("reset_force_sensor", np.random.randint(100))
            time.sleep(0.01)
        time.sleep(1)
        print("Reset")
        self.send_input_to_ur5e("stop", np.random.randint(100))
        time.sleep(0.1)
        self.send_input_to_ur5e("stop", np.random.randint(100))


    def get_ee_given_tip_pose(self,tip_pose):
        return np.hstack(((transformMatrix_from_pose(tip_pose)@self.tooltip_to_ee_transform)[0:3].flatten(),tip_pose[3:6]))

    def get_tip_pose_given_ee(self,ee_pose):
        return np.hstack(((transformMatrix_from_pose(ee_pose)@self.ee_to_tooltip_transform)[0:3].flatten(),ee_pose[3:6]))

    def rate_sleep(self, rate=1.0/125):
        dt = time.time()-self.t_check
        time.sleep(max(0,rate-dt))
        self.t_check = time.time()

    def interpol(self,vec,dt,t):
        i = int(t/dt)
        if type(vec)==list:
            vec = np.array(vec)
        if len(vec.shape)==1:
            try:
                val = (vec[i+1]-vec[i])*(t%dt)/dt+vec[i]
            except:
                val = vec[-1]
                # print("Sequence Finished")
            return val
        else:
            if type(vec)==list:
                vec = np.array(vec) 
            try:
                val = (vec[:,i+1]-vec[:,i])*(t%dt)/dt+vec[:,i]
            except:
                val = vec[:,-1]
                # print("Sequence Fi
            return val

    def move_l(self,pose, vel= 0.05, use_tool = False): 
        # move to a single pose
        current_pose = self.ur5e_actual_state.get_tcp_pose()
        next_pose = np.empty(6)
        # print("Target", pose)  
        if use_tool:
            current_pose = self.get_tip_pose_given_ee(current_pose)
        dp = current_pose[0:3]-pose[0:3]
        total_time = norm(dp)/vel
        t_start = time.time() 
        t = time.time()-t_start 
        
        vec = np.vstack((current_pose,pose)).T 
        # print(vec)

        rv_stack = interpolate_rvs(current_pose[3:6],pose[3:6], 100) # 3x100 rotation vector
        # print(rv_stack)
        print("Start", current_pose)
        print("Target", pose)
        # input("Wait at movel")  

        if use_tool:
            while (norm(pose[0:3]-self.get_tip_pose_given_ee(self.ur5e_actual_state.get_tcp_pose())[0:3])>0.0002) or (norm(rv2rpy(pose[3:6])-rv2rpy(self.ur5e_actual_state.get_tcp_pose()[3:6]))>0.0002):
                t = time.time()-t_start 
                interp_pos = self.interpol(vec,total_time,t)[0:3]
                interp_ori = self.interpol(rv_stack,total_time/100.0,t) 
                next_pose[0:3] = self.get_ee_given_tip_pose(np.hstack((interp_pos,interp_ori)))[0:3]
                next_pose[3:6] = interp_ori 
                self.send_input_to_ur5e("servol",next_pose,self.ur5e_actual_state.get_joint_angles(),[0.1,0.1,1.0/125,0.05,1200]) 
                self.rate_sleep()
        else:
            while (norm(pose[0:3]-self.ur5e_actual_state.get_tcp_pose()[0:3])>0.0002) or (norm(rv2rpy(pose[3:6])-rv2rpy(self.ur5e_actual_state.get_tcp_pose()[3:6]))>0.0002):
        
                t = time.time()-t_start 
                next_pose[0:3] = self.interpol(vec,total_time,t)[0:3]
                next_pose[3:6] = self.interpol(rv_stack,total_time/100.0,t) 
                # print(next_pose)
                self.send_input_to_ur5e("servol",next_pose,self.ur5e_actual_state.get_joint_angles(),[0.1,0.1,1.0/125,0.05,1200]) 
                self.rate_sleep()

        # print("Movel completed")
 
    def servo_l_from_array(self,pose_array, vel= 0.05 , time_override = 0 , use_tool = False):
        # pose array (6xN) : N poses of positions and orientations
        current_pose = self.ur5e_actual_state.get_tcp_pose()
        next_pose = np.empty(6)
        N = pose_array.shape[1]

        if use_tool:
            for i in range(N):
                pose_array[:,i] = self.get_ee_given_tip_pose(pose_array[:,i])

        if (norm(pose_array[0:3,0]-self.ur5e_actual_state.get_tcp_pose()[0:3])>0.0002) or (norm(rv2rpy(pose_array[3:6,0])-rv2rpy(self.ur5e_actual_state.get_tcp_pose()[3:6]))>0.0002):
            input("Not at start position. Press any key to move to start")
            self.move_l(pose_array[:,0])

        dp = np.sum(np.linalg.norm(np.diff(pose_array[0:3,:],axis=1),axis=0))
        total_time = norm(dp)/vel
        dt = total_time/N

        if time_override != 0:
            dt = time_override/N
        pose = pose_array[:,-1]
        print(pose)
        t_start = time.time() 
        t = time.time()-t_start 
 
        while (norm(pose[0:3]-self.ur5e_actual_state.get_tcp_pose()[0:3])>0.0002) or (norm(rv2rpy(pose[3:6])-rv2rpy(self.ur5e_actual_state.get_tcp_pose()[3:6]))>0.0002):
            t = time.time()-t_start 
            next_pose = self.interpol(pose_array,dt,t)  
            self.send_input_to_ur5e("servol",next_pose,self.ur5e_actual_state.get_joint_angles(),[0.1,0.1,1.0/125,0.05,1200]) 
            self.rate_sleep()
 

    def speedl(self,vel):
        next_pose = self.ur5e_actual_state.get_tcp_pose()+vel*0.03#*self.loop_rate*10
        self.send_input_to_ur5e("servol",next_pose,self.ur5e_actual_state.get_joint_angles(),[0.1,0.1,1.0/125,0.05,1200]) 

        
    def send_input_to_ur5e(self, command_type, *inputs):
        self.low_level_control_input.command_type.data = command_type
        self.low_level_control_input.inputs.data =  np.hstack(inputs)
        # print("Controller node 115", command_type,np.hstack(inputs))
        self.low_level_control_input_publisher.publish(self.low_level_control_input)
    
    def go_to_cell(self, cell_number,vel = 0.3):
        cp = self.cell_pos[cell_number-1]
        # print(cp)
        pose = np.array([cp[0],cp[1],self.cell_home[2],self.cell_home[3],self.cell_home[4],self.cell_home[5]])
        print(f"Cell number {cell_number} at {pose}") 
 
        self.move_l(pose, vel= vel)


    def go_to_xypos_at_cell_height(self, pos,vel = 0.1): 
        pose = np.array([pos[0],pos[1],self.cell_home[2],self.cell_home[3],self.cell_home[4],self.cell_home[5]])

        print(f"Going to pose: {pose}") 
 
        self.move_l(pose, vel= vel)

    def wait_for_data(self):
        while not self.data_received: 
            time.sleep(0.01)  

    def move_arc(self,pose,vel=0.03,height=0.007,time_override=0):

        dp = pose[0:3]-self.ur5e_actual_state.get_tcp_pose()[0:3]
        start_pose = self.ur5e_actual_state.get_tcp_pose()
        total_time = norm(dp)/vel
        if time_override != 0:
            total_time = time_override
        t_start = time.time()
        t = 0 
        vec = np.vstack((self.ur5e_actual_state.get_tcp_pose(),pose)).T 
 
        while norm(pose-self.ur5e_actual_state.get_tcp_pose())>0.0002:
            t = time.time()-t_start
            x_interp = self.interpol([start_pose[0],pose[0]],total_time,t)
            y_interp = self.interpol([start_pose[1],pose[1]],total_time,t)
            z_interp = self.interpol([start_pose[2],start_pose[2]+height,start_pose[2]+height,start_pose[2]+height,start_pose[2]+height ,pose[2]],total_time/5,t)
            rx_interp = self.interpol([start_pose[3],pose[3]],total_time,t)
            ry_interp = self.interpol([start_pose[4],pose[4]],total_time,t)
            rz_interp = self.interpol([start_pose[5],pose[5]],total_time,t)
            # next_pos = start_pose + np.array([x_interp,y_interp,z_interp,0,0,0])
            next_pos = np.array([x_interp,y_interp,z_interp,rx_interp,ry_interp,rz_interp])
            # print(start_pose)
            # print(next_pos)
            self.send_input_to_ur5e("servol",next_pos,self.ur5e_actual_state.get_joint_angles(),[0.1,0.1,1.0/125,0.05,1200])


    def lower_until_contact(self,desired_force = 2.0):
        contact_made = False
        print("Lowering")
        self.desired_force = desired_force
        while not(contact_made):
            # a = time.time() 
            # if self.ur5e_actual_state.get_filtered_tcp_force()[2]>3.0:
            if self.ur5e_actual_state.get_tcp_force()[2]>self.desired_force:
            # if self.ur5e_actual_state.get_filtered_tcp_force()[2]>self.desired_force:
                print(f"Contact Made at EE: {self.ur5e_actual_state.get_tcp_pose()}")
                contact_made = True
                vel = np.array([0,0, -0.001,0,0,0])
                self.contact_pose = self.ur5e_actual_state.get_tcp_pose()#+np.array([0,0,-0.0005,0,0,0])
                self.contact_height = self.contact_pose[2]
                self.tip_pos_base_frame = (transformMatrix_from_pose(self.ur5e_actual_state.get_tcp_pose())@self.ee_to_tooltip_transform)[0:3].flatten()
                
                print(f"Contact Made at tip pos: {self.tip_pos_base_frame}")
                self.ee_given_tip_pose= transformMatrix_from_pose(np.hstack((self.tip_pos_base_frame,rpy2rv(pi,0,0))))@self.tooltip_to_ee_transform
                print(f"calc to eee: {self.ee_given_tip_pose[0:3].flatten()}")
                self.send_input_to_ur5e("servol",self.contact_pose,self.ur5e_actual_state.get_joint_angles(),[0.1,0.1,1.0/125,0.05,1200])
            else:
                vel = np.array([0,0,-0.01,0,0,0])
                self.speedl(vel) 
                self.rate_sleep() 

    def run_main_loop(self):  
        print("Starting controller loop")
        contacted = False
        returnup = 0 
        while not self.data_received:
            # print("Waiting for Data")
            time.sleep(0.01) 
        # self.initialize_position()
        self.move_to_initial_pose(0.1) 
        self.move_up_from_current_pose(0.05, 0.1,use_tool=True) #463
        # input('wait')
        self.move_l([-0.458,-0.07,0.5,1.8,-1.8,-0.67],use_tool=True)
        input('wait') #Target [-0.39206 -0.06926  0.57517  1.8     -1.8     -0.67   ]

        self.servo_l_from_array(np.array([[-0.458,-0.07,0.5,1.8,-1.8,-0.67],[-0.458,-0.07,0.3,0.8,-0.8,-0.67]]).T,vel=0.01,use_tool=True)
        if self.stay_at_initial:
            input()
 
        self.task_completed_publisher.publish(True) 

if __name__ == "__main__":
    baseline_arm_controller()
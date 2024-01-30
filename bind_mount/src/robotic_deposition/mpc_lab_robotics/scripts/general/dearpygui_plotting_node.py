#!/usr/bin/env python3
"""
Class for visualizing the UR5e robot arm data
Author: Tony Zheng
"""
import numpy as np
import pandas as pd
import dearpygui.dearpygui as dpg
import rospy
import signal 
import datetime
import time
import os
import sys

from numpy.linalg import norm
from math import pi
from mpc_lab_robotics.infrastructure.plotting.dearpygui_class import DearPyGuiCreator 
from mpc_lab_robotics.infrastructure.robot.ur_rtde_state import ur_rtde_state, ur_rtde_state_history

from std_msgs.msg import Bool
from robotic_deposition.msg import ur5e_data

class dearpygui_plotting_node():
    def __init__(self, use_ros = True):
        
        self.pygui = DearPyGuiCreator()
        self.add_pygui_plots()

        for tag in self.pygui.plot_tags:
            self.pygui.update_item_label(tag+'/x_axis', 'Time (s)')

        self.add_pygui_lines(tag_prefix="/actual")
        # self.add_pygui_lines(tag_prefix="/commanded")


        self.actual_state = ur_rtde_state_history()
        self.commanded_state = ur_rtde_state_history()
        self.finished_task = False

        # print(self.pygui.get_tag_names())
        # self.pygui.print_tag_names()

        timewidth = 20 #10s window
        self.pygui.set_all_x_axis(0, timewidth)
        # dpg.start_dearpygui()
        # dpg.destroy_context()
        signal.signal(signal.SIGINT, self.exit_program)  

        if use_ros:
            rospy.init_node('dearpygui_plotting_node', anonymous=False)
            self.loop_rate = 60
            self.rate =  rospy.Rate(self.loop_rate)

            rospy.Subscriber('/ur5e/rtde_sensor_data/actual', ur5e_data, self.callback_actual_ur5e_data, queue_size = 10)
            rospy.Subscriber('/ur5e/rtde_sensor_data/target', ur5e_data, self.callback_commanded_ur5e_data, queue_size = 10)
            rospy.Subscriber('/finished_task', Bool, self.callback_task_completion, queue_size = 10)

            print("paint_node ROS initialized")
            self.data_received = False

            t = []
            d = []
            tstart = time.time()
            t.append(time.time()-tstart)
            d.append(np.random.rand(6))
            while not self.data_received:
                # print("Waiting for Data")
                time.sleep(0.01) 
                self.pygui.render_frame()

            print("Starting plotting loop") 
            while not rospy.is_shutdown():  
                # timewidth = 20 #10s window
                # print(len(t),self.j.shape)
                self.update_pygui_states(self.actual_state, tag_prefix="/actual",t=timewidth)
                # self.update_pygui_states(self.commanded_state, tag_prefix="/commanded")
                if self.actual_state.time_list[-1]> timewidth:
                    self.pygui.set_all_x_axis(self.actual_state.time_list[-1] - timewidth, self.actual_state.time_list[-1])
                self.pygui.render_frame() 
                if self.finished_task:
                    print("Task Completed")
                    break
                self.rate.sleep() 
        else:
            while dpg.is_dearpygui_running():
                self.pygui.render_frame()
        
        self.exit_program()

    def exit_program(self, *args, **kwargs):
        print("Exiting Plotting Node") 
        sys.exit()

    def update_pygui_states(self, rtde_state, tag_prefix, t=0):
        if t==0:
            self.pygui.update_data(tag_name = tag_prefix+'/joint_angles', label=tag_prefix+'j', data_x =  rtde_state.time_list, data_y = rtde_state.joint_angle_list[:,0:rtde_state.i]) 
            self.pygui.update_data(tag_name = tag_prefix+'/joint_velocities', label='dj', data_x =  rtde_state.time_list, data_y = rtde_state.joint_velocities_list[:,0:rtde_state.i]) 
            self.pygui.update_data(tag_name = tag_prefix+'/tcp_pose', label='P', data_x =  rtde_state.time_list, data_y = rtde_state.tcp_pose_list[:,0:rtde_state.i]) 
            self.pygui.update_data(tag_name = tag_prefix+'/tcp_speed', label='V', data_x =  rtde_state.time_list, data_y = rtde_state.tcp_speed_list[:,0:rtde_state.i]) 
            self.pygui.update_data(tag_name = tag_prefix+'/tcp_force', label='F', data_x =  rtde_state.time_list, data_y = rtde_state.tcp_force_list[:,0:rtde_state.i]) 
            self.pygui.update_data(tag_name = tag_prefix+'/filtered_tcp_force', label='F', data_x =  rtde_state.time_list, data_y = rtde_state.filtered_tcp_force_list[:,0:rtde_state.i]) 
        else:
            imin = max(0,rtde_state.i-t*500)
            self.pygui.update_data(tag_name = tag_prefix+'/joint_angles', label=tag_prefix+'j', data_x =  rtde_state.time_list[imin:rtde_state.i], data_y = rtde_state.joint_angle_list[:,imin:rtde_state.i]) 
            self.pygui.update_data(tag_name = tag_prefix+'/joint_velocities', label='dj', data_x =  rtde_state.time_list[imin:rtde_state.i], data_y = rtde_state.joint_velocities_list[:,imin:rtde_state.i]) 
            self.pygui.update_data(tag_name = tag_prefix+'/tcp_pose', label='P', data_x =  rtde_state.time_list[imin:rtde_state.i], data_y = rtde_state.tcp_pose_list[:,imin:rtde_state.i]) 
            self.pygui.update_data(tag_name = tag_prefix+'/tcp_speed', label='V', data_x =  rtde_state.time_list[imin:rtde_state.i], data_y = rtde_state.tcp_speed_list[:,imin:rtde_state.i]) 
            self.pygui.update_data(tag_name = tag_prefix+'/tcp_force', label='F', data_x =  rtde_state.time_list[imin:rtde_state.i], data_y = rtde_state.tcp_force_list[:,imin:rtde_state.i]) 
            self.pygui.update_data(tag_name = tag_prefix+'/filtered_tcp_force', label='F', data_x =  rtde_state.time_list[imin:rtde_state.i], data_y = rtde_state.filtered_tcp_force_list[:,imin:rtde_state.i]) 
        
    def add_pygui_lines(self,tag_prefix):
        self.pygui.add_line([], [], tag_name=tag_prefix+"/joint_angles"+"_lines", label=tag_prefix+'j', parent_name = "/joint_angles"+"_tab")
        self.pygui.add_line([], [], tag_name=tag_prefix+"/joint_velocities"+"_lines", label='dj', parent_name = "/joint_velocities"+"_tab")
        self.pygui.add_line([], [], tag_name=tag_prefix+"/tcp_pose"+"_lines", label='P', parent_name = "/tcp_pose"+"_tab")
        self.pygui.add_line([], [], tag_name=tag_prefix+"/tcp_speed"+"_lines", label='V', parent_name = "/tcp_speed"+"_tab")
        self.pygui.add_line([], [], tag_name=tag_prefix+"/tcp_force"+"_lines", label='F', parent_name = "/tcp_force"+"_tab")
        self.pygui.add_line([], [], tag_name=tag_prefix+"/filtered_tcp_force"+"_lines", label='F', parent_name = "/tcp_force"+"_tab")
        self.pygui.set_theme_for_lines("/filtered_tcp_force")

    def add_pygui_lines_commanded(self,tag_prefix):
        self.pygui.add_line([], [], tag_name=tag_prefix+"/joint_angles"+"_lines", label=tag_prefix+'j', parent_name = "/joint_angles"+"_tab")
        # self.pygui.add_line([], [], tag_name=tag_prefix+"/joint_velocities"+"_lines", label='dj', parent_name = "/joint_velocities"+"_tab")
        # self.pygui.add_line([], [], tag_name=tag_prefix+"/tcp_pose"+"_lines", label='P', parent_name = "/tcp_pose"+"_tab")
        # self.pygui.add_line([], [], tag_name=tag_prefix+"/tcp_speed"+"_lines", label='V', parent_name = "/tcp_speed"+"_tab")
        # self.pygui.add_line([], [], tag_name=tag_prefix+"/tcp_force"+"_lines", label='F', parent_name = "/tcp_force"+"_tab")
        # self.pygui.add_line([], [], tag_name=tag_prefix+"/filtered_tcp_force"+"_lines", label='F', parent_name = "/tcp_force"+"_tab")
        # self.pygui.set_theme_for_lines("/filtered_tcp_force")

    def add_pygui_plots(self):
        self.pygui.generate_plot_in_tab(tag_name = "/joint_angles"+"_tab",tab_label="Joint Angles",tab_parent="Tab1", plot_label="Joint Angles")
        self.pygui.generate_plot_in_tab(tag_name = "/joint_velocities"+"_tab",tab_label="Joint Velocities",tab_parent="Tab1", plot_label="Joint Velocities")
        self.pygui.generate_subplot_in_tab(tag_name = "/tcp_pose"+"_tab",tab_label="TCP Pose",tab_parent="Tab1", plot_label="TCP Pose")
        self.pygui.generate_subplot_in_tab(tag_name = "/tcp_speed"+"_tab",tab_label="TCP Velocities",tab_parent="Tab1", plot_label="TCP Velocities")
        self.pygui.generate_subplot_in_tab(tag_name = "/tcp_force"+"_tab",tab_label="TCP Forces",tab_parent="Tab1", plot_label="TCP Forces") 

    def callback_actual_ur5e_data(self,msg):   
        self.actual_state.add_data_from_msg(msg)
        self.data_received = True

    def callback_commanded_ur5e_data(self,msg):   
        self.commanded_state.add_data_from_msg(msg)
        # self.data_received = True

    def callback_task_completion(self,msg):
        self.finished_task = msg.data

if __name__ == "__main__": 
    dearpygui_plotting_node(use_ros=True)

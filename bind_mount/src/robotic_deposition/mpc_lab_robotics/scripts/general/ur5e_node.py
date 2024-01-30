#!/usr/bin/env python3
"""
Class for using the UR5e with ROS
Author: Tony Zheng
"""
import numpy as np
from numpy.linalg import norm
from math import pi
import time
import rospy
import signal

from mpc_lab_robotics.infrastructure.robot.universal_robot import UR_Robot 
from robotic_deposition.msg import ur5e_data, ur5e_control 

from std_msgs.msg import Bool
np.set_printoptions(suppress=True,precision=5,linewidth = np.inf)

class ur5e_node():
    def __init__(self, frequency = 125):
        try:
            self.sim = rospy.get_param('sim')
            self.load_program_and_play = rospy.get_param('load_program_and_play')
        except:
            self.sim = True
            self.load_program_and_play = True  
        
        self.ur5e = UR_Robot(sim=self.sim,load_program_and_play = self.load_program_and_play, program_name = 'rtde_control_loop.urp',frequency = frequency)  

        self.frequency = frequency
        self.selection_vector = np.array([0,0,1,0,0,0])
        
        time.sleep(0.1)

        self.initialize_ros()
        self.finished_task = False
        self.run_main_loop() 

    def stop_robot(self, *args, **kwargs):
        print("Stopping robot")
        
        self.ur5e.stop_robot()
        # sys.exit(0)

    def initialize_ros(self):
        rospy.init_node('ur5e_node', anonymous=False)
        self.loop_rate = self.frequency
        self.rate =  rospy.Rate(self.loop_rate)

        signal.signal(signal.SIGINT, self.stop_robot)   

        self.ur5e_actual_data = ur5e_data()
        self.ur5e_target_data = ur5e_data()
        self.ur5e_actual_data_publisher = rospy.Publisher('/ur5e/rtde_sensor_data/actual', ur5e_data, queue_size = 1)
        self.ur5e_commanded_data_publisher = rospy.Publisher('/ur5e/rtde_sensor_data/target', ur5e_data, queue_size = 1)

        rospy.Subscriber('/controllers/ur5e_low_level_control', ur5e_control, self.callback_commanded_ur5e_data, queue_size = 10)
        rospy.Subscriber('/finished_task', Bool, self.callback_task_completion, queue_size = 10)
        self.low_level_control_input = ur5e_control()
        print("ur5e_node ROS initialized")

    def callback_commanded_ur5e_data(self,msg):
        self.low_level_control_input = msg
        self.ur5e.send_command_type(msg.command_type.data, list(msg.inputs.data))

    def run_main_loop(self): 
        tstart = time.time() 
        print("Starting ur5e loop")
        # force_data = []
        # i = 0
        while not rospy.is_shutdown(): 
            state = self.ur5e.get_state()
            if state is not None:
                # ur5e.rtde_kick_watchdog()
                # print("--------------------------------------")
                # print(self.ur5e.get_actual_joint_angles())
                # print(self.ur5e.get_actual_tcp_pose())
                # print(self.ur5e.get_actual_tcp_force())
                # print(ur5e.get_program_status())
                # print(ur5e.get_robot_operational_status())
                # print(ur5e.socket_get_programState())
                # print(ur5e.socket_get_robotmode())
                # print(ur5e.socket_get_safetystatus())
                # print("ur", self.ur5e.get_robot_is_steady())
                
                self.ur5e_actual_data.joint_angles.data = self.ur5e.get_actual_joint_angles()
                self.ur5e_actual_data.joint_velocities.data = self.ur5e.get_actual_joint_velocities()
                self.ur5e_actual_data.tcp_pose.data = self.ur5e.get_actual_tcp_pose()
                self.ur5e_actual_data.tcp_speed.data = self.ur5e.get_actual_tcp_speed()
                self.ur5e_actual_data.tcp_force.data = self.ur5e.get_actual_tcp_force()
                self.ur5e_actual_data.is_steady.data = self.ur5e.get_robot_is_steady()

                self.ur5e_target_data.joint_angles.data = self.ur5e.get_target_joint_angles()
                self.ur5e_target_data.joint_velocities.data = self.ur5e.get_target_joint_velocities()
                self.ur5e_target_data.tcp_pose.data = self.ur5e.get_target_tcp_pose()
                self.ur5e_target_data.tcp_speed.data = self.ur5e.get_target_tcp_speed()

                self.ur5e_target_data.time.data = time.time()-tstart
                self.ur5e_actual_data.time.data = time.time()-tstart
                self.ur5e_actual_data_publisher.publish(self.ur5e_actual_data)
                self.ur5e_commanded_data_publisher.publish(self.ur5e_target_data)
                # force_data.append(self.ur5e_actual_data.tcp_force.data)
                # i+=1
                # print(i)
                # if i==10000:
                #     df = pd.DataFrame(np.vstack(force_data), columns=['Fx', 'Fy', 'Fz', 'Tx', 'Ty', 'Tz', ])

                #     df.to_csv(os.path.dirname(os.path.abspath(__file__))+'/force_data.csv')
                #     pdb.set_trace()
                if self.finished_task:
                    break
            self.rate.sleep()
        print("Stopping ur5e_node")
        # self.ur5e.stop_robot()
        # sys.exit()

    def callback_task_completion(self,msg):
        self.finished_task = msg.data

if __name__ == "__main__":
    ur5e_node()
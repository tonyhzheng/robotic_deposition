import numpy as np
import time 

class ur_rtde_state_history():
    def __init__(self, n = 125*60*60*10):
        self.time_list = [] 
        self.joint_angle_list = np.empty([6, n])
        self.joint_velocities_list = np.empty([6, n])
        self.tcp_pose_list = np.empty([6, n])
        self.tcp_speed_list = np.empty([6, n])
        self.tcp_force_list = np.empty([6, n])
        self.filtered_tcp_force_list = np.empty([6, n])
        self.is_steady = False
        self.i = 0
        self.n = n

        #Low pass filter with cutoff freq 4
        self.b = [0.02451657, 0.02451657]
        self.a = [-1. ,0.95096685]

    def add_data_from_msg(self,msg): 
        a = time.time()
        if self.i>=self.n:
            self.joint_angle_list = np.hstack((self.joint_angle_list, np.empty([6, self.n])))
            self.joint_velocities_list = np.hstack((self.joint_velocities_list, np.empty([6, self.n])))
            self.tcp_pose_list = np.hstack((self.tcp_pose_list, np.empty([6, self.n])))
            self.tcp_speed_list = np.hstack((self.tcp_speed_list, np.empty([6, self.n])))
            self.tcp_force_list = np.hstack((self.tcp_force_list, np.empty([6, self.n])))
            self.filtered_tcp_force_list = np.hstack((self.filtered_tcp_force_list, np.empty([6, self.n])))
            self.n += self.n
            print("EXPANDING",self.n) 

        self.time_list.append(msg.time.data)
        self.joint_angle_list[:,self.i] = msg.joint_angles.data
        self.joint_velocities_list[:,self.i] = msg.joint_velocities.data
        self.tcp_pose_list[:,self.i] = msg.tcp_pose.data
        self.tcp_speed_list[:,self.i] = msg.tcp_speed.data
        if msg.tcp_force.data :
            self.tcp_force_list[:,self.i] = msg.tcp_force.data 

            if self.i == 0:
                self.filtered_tcp_force_list[:,self.i] = msg.tcp_force.data 
            else:
                self.filtered_tcp_force_list[:,self.i] = self.a[1]*self.filtered_tcp_force_list[:,self.i-1] + self.b[0]*self.tcp_force_list[:,self.i] + self.b[1]*self.tcp_force_list[:,self.i-1]
        self.is_steady = msg.is_steady.data
        
        self.i +=1 
        # print(a,time.time()-a,self.i, self.n)

    def get_timestamp(self):
        return self.time_list[-1]
        
    def get_robot_is_steady(self):
        return self.is_steady

    def get_joint_angles(self):
        return self.joint_angle_list[:,self.i-1]

    def get_joint_velocities(self):
        return self.joint_velocities_list[:,self.i-1]

    def get_tcp_pose(self):
        return self.tcp_pose_list[:,self.i-1]

    def get_tcp_speed(self):
        return self.tcp_speed_list[:,self.i-1]

    def get_tcp_force(self):
        return self.tcp_force_list[:,self.i-1]
        
    def get_filtered_tcp_force(self):
        return self.filtered_tcp_force_list[:,self.i-1]


class ur_rtde_state():
    def __init__(self, ):
        self.time = 0
        self.joint_angle = np.empty(6)
        self.joint_velocities = np.empty(6)
        self.tcp_pose = np.empty(6)
        self.tcp_speed = np.empty(6)
        self.tcp_force = np.empty(6)
        self.prev_tcp_force = np.empty(6)
        self.filtered_tcp_force = np.empty(6)
        self.is_steady = False
        self.i = 0 

        #Low pass filter with cutoff freq 4
        self.b = [0.02451657, 0.02451657]
        self.a = [-1. ,0.95096685]

    def add_data_from_msg(self,msg):  
        self.time = msg.time.data
        self.is_steady = msg.is_steady.data
        self.joint_angle[:] = msg.joint_angles.data
        self.joint_velocities[:] = msg.joint_velocities.data
        self.tcp_pose[:] = msg.tcp_pose.data
        self.tcp_speed[:] = msg.tcp_speed.data
        # print(self.tcp_force)
        # print(msg.tcp_force.data)
        self.tcp_force[:] = msg.tcp_force.data 
        
        # if msg.tcp_force.data :
        if self.i == 0:
            self.filtered_tcp_force[:] = msg.tcp_force.data 
        else: 
            self.filtered_tcp_force = self.a[1]*self.filtered_tcp_force+ self.b[0]*self.tcp_force + self.b[1]*self.prev_tcp_force
        self.prev_tcp_force[:] = self.tcp_force
        self.i +=1  

    def get_timestamp(self):
        return self.time
        
    def get_robot_is_steady(self):
        return self.is_steady

    def get_joint_angles(self):
        return self.joint_angle

    def get_joint_velocities(self):
        return self.joint_velocities

    def get_tcp_pose(self):
        return self.tcp_pose

    def get_tcp_speed(self):
        return self.tcp_speed

    def get_tcp_force(self):
        return self.tcp_force
        
    def get_filtered_tcp_force(self):
        return self.filtered_tcp_force
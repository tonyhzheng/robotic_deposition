import rtdeState
import csv
import time
import copy
import sys
import numpy as np

from paint_robotics.utils.jacobian_lib import *
np.set_printoptions(suppress=True,precision=5,linewidth = np.inf)
name = 'path500.csv'
ROBOT_HOST = '192.168.1.2'
# ROBOT_HOST = 'localhost'
ROBOT_PORT = 30004
config_filename = 'rtdeCommand.xml'


def list_to_set_q(set_q, list):
    #set_q doesn't need to be passed out as variable for the changes stay
    # python usese call by object
    # set_q originally becomes a name for the dict from rtde
    # so anything that happens to set_q affects thee original dict
    # here set_q becomes reassigned as the name for set_q2 thus another changes after dont stay the same
    for i in range(0, len(list)):
        set_q2 = copy.deepcopy(set_q)
        set_q2.__dict__["input_double_register_%i" % i] = list[i]
        # set_q.input_double_register_0 = 10
        set_q = set_q2
    return set_q2

def print_q(set_q):
    print(set_q.input_double_register_0)

with open(name, 'rt') as csvfile:
    reader = csv.reader(csvfile, delimiter=',')
    q1 = []
    q2 = []
    q3 = []
    q4 = []
    q5 = []
    q6 = []
    for row in reader:
        q1.append(float(row[0]))
        q2.append(float(row[1]))
        q3.append(float(row[2]))
        q4.append(float(row[3]))
        q5.append(float(row[4]))
        q6.append(float(row[5]))

rtde = rtdeState.RtdeState(ROBOT_HOST, config_filename)
print("robot initialized")
rtde.initialize()
rtde.set_q.input_double_register_0 = q1[0]
rtde.set_q.input_double_register_1 = q2[0]
rtde.set_q.input_double_register_2 = q3[0]
rtde.set_q.input_double_register_3 = q4[0]
rtde.set_q.input_double_register_4 = q5[0]
rtde.set_q.input_double_register_5 = q6[0]
rtde.servo.input_int_register_0 = 0

set_q2 = copy.deepcopy(rtde.set_q)
print(rtde.set_q.input_double_register_0)
list_to_set_q(set_q2, [1,2,3,4,5,6])
print(rtde.set_q.input_double_register_0) 
print(rtde.set_q.input_double_register_0)
list_to_set_q(rtde.set_q, [1,2,3,4,5,6])
print(rtde.set_q.input_double_register_0) 
print_q(list_to_set_q(rtde.set_q, [1,2,3,4,5,6]))
# Wait for program to be started and ready.
state = rtde.receive()
# print(state.output_int_register_0)
# sys.exit()

while state.output_int_register_0 != 1:
    a=time.time()
    state = rtde.receive()
    # print(1/(time.time()-a))
    # print(state.robot_status_bits, state.safety_status_bits, state.safety_mode, state.safety_status)
    torques = np.array([state.output_double_register_0,state.output_double_register_1,state.output_double_register_2,state.output_double_register_3,state.output_double_register_4,state.output_double_register_5])
    measured_q = state.actual_q
    applied_F = runJac(measured_q,0)@torques
    print(applied_F)
    time.sleep(0.01)
sys.exit()
# Send command to robot to begin servoing.
rtde.servo.input_int_register_0 = 1
rtde.con.send(rtde.servo)
time.sleep(0.01)

# Main control loop. Receive an output packet from the robot and then send the next joint positions.
for i in range(len(q1)):
    state = rtde.receive()
    print(state.output_double_register_0)
    print(state.safety_status_bits)
    # list_to_set_q(rtde.set_q, [q1[i], q2[i], q3[i], q4[i], q5[i]])
    list_to_set_q(rtde.set_q, [q1[i], q2[i], q3[i], q4[i], q5[i], q6[i]])
    rtde.con.send(rtde.set_q)

# Stop servoing.
rtde.servo.input_int_register_0 = 0
rtde.con.send(rtde.servo)
time.sleep(0.01)
rtde.con.send_pause()
rtde.con.disconnect()

"""
Library of useful functions

Author: Tony Zheng 
"""
from asyncio import wait
import numpy as np
from numpy import sign,eye, dot
from numpy.linalg import norm,inv,det
import quaternion
from math import pi,sqrt,atan2,sin,cos, acos 

np.set_printoptions(suppress=True,precision=5,linewidth = np.inf)

def skewSym(v): 
    M = np.array([[0,-float(v[2]),float(v[1])],
                  [float(v[2]),0,-float(v[0])],
                  [-float(v[1]),float(v[0]),0]]) 
    return M

# Rotation Matrix Equations
def Rx(roll):
    c,s = cos(roll),sin(roll)
    R = np.array([[1,0,0],[0,c,-s],[0,s,c]])
    return R

def Ry(pitch):
    c,s = cos(pitch),sin(pitch)
    R = np.array([[c,0,s],[0,1,0],[-s, 0, c]])
    return R

def Rz(yaw):
    c,s = cos(yaw),sin(yaw)
    R = np.array([[c,-s,0],[s,c,0],[0,0,1]])
    return R

def R_XYZ(roll,pitch,yaw):
    # fixed angle (the rotation axis are fixed) multiply from the left side, Rz*Ry*Rx*R_current
    # ZYX euler angle (moving axis rotations) multiply from the right side, R_current*Rz*Ry*Rx
    
    return Rz(yaw)@Ry(pitch)@Rx(roll)

def R_ZYX(roll,pitch,yaw):
    return Rx(roll)@Ry(pitch)@Rz(yaw)

def quat_to_rv(quat):
    q0,q1,q2,q3 = quat
    theta = 2*acos(q0)
    if theta != 0:
        omega = np.array([q1,q2,q3])/sin(theta/2)
    else:
        omega = np.zeros(3)
    return theta*omega

def rv_to_w_omega(rv):
    theta = np.linalg.norm(rv)
    omega = rv/theta
    return theta,omega

def rv_to_quat(rv):
    theta,omega = rv_to_w_omega(rv)
    q0 = cos(theta/2)
    q1,q2,q3 = omega*sin(theta/2)
    return np.array([q0,q1,q2,q3])

def rpy2rv(roll,pitch,yaw):
    R = R_XYZ(roll,pitch,yaw) 
    return matrix_to_rv(R)

def matrix_to_quat(R): 
    q_squared_vec = 0.25*np.array([[1,1,1,1],[1,-1,-1,1],[-1,1,-1,1],[-1,-1,1,1]])@np.array([ R[0,0],R[1,1],R[2,2],1]).reshape(-1,1)
    max_idx = np.argmax(q_squared_vec)
    # print(q_squared_vec)
    # print(max_idx)
    if max_idx == 0:
        q0 = sqrt(q_squared_vec[0])
        q1 = (R[2,1]-R[1,2])/(4*q0)
        q2 = (R[0,2]-R[2,0])/(4*q0)
        q3 = (R[1,0]-R[0,1])/(4*q0)
    elif max_idx ==1:
        q1 = sqrt(q_squared_vec[1])
        q0 = (R[2,1]-R[1,2])/(4*q1)
        q2 = (R[1,0]+R[0,1])/(4*q1)
        q3 = (R[0,2]+R[2,0])/(4*q1) 
    elif max_idx ==2:
        q2 = sqrt(q_squared_vec[1])
        q0 = (R[0,2]-R[2,0])/(4*q2)
        q1 = (R[1,0]+R[0,1])/(4*q2)
        q3 = (R[2,1]+R[1,2])/(4*q2)
    elif max_idx ==3:
        q3 = sqrt(q_squared_vec[1])
        q0 = (R[1,0]-R[0,1])/(4*q3)
        q1 = (R[0,2]+R[2,0])/(4*q3)
        q2 = (R[2,1]+R[1,2])/(4*q3)
    else:
        print("error")
    return np.array([q0,q1,q2,q3])

def slerp_shoemake(quat_1,quat_2,t):
    theta = acos(dot(quat_1/norm(quat_1),quat_2/norm(quat_2))) 
    s_theta = sin(theta)
    return sin((1-t)*theta)/s_theta*quat_1+sin(t*theta)/s_theta*quat_2

def slerp_standard(quat_1,quat_2,t_slerp):
    return quaternion.slerp_evaluate(np.quaternion(quat_1[0],quat_1[1],quat_1[2],quat_1[3]),np.quaternion(quat_2[0],quat_2[1],quat_2[2],quat_2[3]),t_slerp)

def interpolate_rvs(rv_start,rv_end,N):
    quat_start = rv_to_quat(rv_start)
    quat_end = rv_to_quat(rv_end)
    return interpolate_quats(quat_start,quat_end, N)


def interpolate_eulers(euler_start,euler_end, N):
    roll_start, pitch_start, yaw_start = euler_start
    roll_end, pitch_end, yaw_end = euler_end

    rot_start = R_XYZ(roll_start, pitch_start, yaw_start)
    rot_end = R_XYZ(roll_end, pitch_end, yaw_end)
    return interpolate_rotations(rot_start,rot_end, N)

def interpolate_rotations(rot_start,rot_end, N):
    quat_start = matrix_to_quat(rot_start)
    quat_end = matrix_to_quat(rot_end)

    return interpolate_quats(quat_start,quat_end, N)

def interpolate_quats(quat_start,quat_end, N): 
    rv_stack = np.zeros([3,N])
    t_array = np.linspace(0,1,N)
    for i in range(N):
        t_slerp = t_array[i]
        quat_t = slerp_standard(quat_start,quat_end,t_slerp)
        rv = quat_to_rv([quat_t.w,quat_t.x,quat_t.y,quat_t.z])  
        rv_stack[:,i] = rv 
    return rv_stack

def matrix_to_rv(R): 
    u = np.array([R[2,1]-R[1,2],R[0,2]-R[2,0],R[1,0]-R[0,1]])/np.linalg.norm(np.array([R[2,1]-R[1,2],R[0,2]-R[2,0],R[1,0]-R[0,1]]))
    # u = np.array([R[2,1]-R[1,2],R[0,2]-R[2,0],R[1,0]-R[0,1]])/fast_norm(np.array([R[2,1]-R[1,2],R[0,2]-R[2,0],R[1,0]-R[0,1]]))

    return u*acos((np.trace(R)-1)/2) 

def fast_norm(a):
    return sqrt(sum(a**2))    

def rv_to_matrix(rv):
    if not np.any(rv):
        return np.eye(3)
    rodrigues = True
    if rodrigues:
        theta,omega = rv_to_w_omega(rv)
        w1 = omega[0]
        w2 = omega[1]
        w3 = omega[2]
        ctheta = cos(theta)
        vtheta = 1-ctheta
        stheta = sin(theta)
        R = np.array([[(w1**2)*vtheta+ctheta, w1*w2*vtheta-w3*stheta, w1*w3*vtheta+w2*stheta],
                      [ w1*w2*vtheta+w3*stheta,(w2**2)*vtheta+ctheta,w2*w3*vtheta-w1*stheta],
                      [ w1*w3*vtheta-w2*stheta, w2*w3*vtheta+w1*stheta,(w3**2)*vtheta+ctheta],])
        return R
    else:
        [roll,pitch,yaw] = rv2rpy(rv)
        return R_XYZ(roll,pitch,yaw)

def quat_to_matrix(quat):
    q0 = quat[0]
    q1 = quat[1]
    q2 = quat[2]
    q3 = quat[3]
    R = np.array([[1-2*(q2**2+q3**2), 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2)],    
                [2*(q1*q2+q0*q3),1-2*(q1**2+q3**2), 2*(q2*q3-q0*q1)],    
                [2*(q1*q3-q0*q2),2*(q2*q3+q0*q1), 1-2*(q1**2+q2**2)]])
    return R
def rpy2rv_UR(roll,pitch,yaw):
    # https://forum.universal-robots.com/t/pose-rotation-order/223
    alpha = yaw
    beta = pitch
    gamma = roll

    ca = cos(alpha)
    cb = cos(beta)
    cg = cos(gamma)
    sa = sin(alpha)
    sb = sin(beta)
    sg = sin(gamma)

    r11 = ca*cb
    r12 = ca*sb*sg-sa*cg
    r13 = ca*sb*cg+sa*sg
    r21 = sa*cb
    r22 = sa*sb*sg+ca*cg
    r23 = sa*sb*cg-ca*sg
    r31 = -sb
    r32 = cb*sg
    r33 = cb*cg

    theta = acos((r11+r22+r33-1)/2)
    sth = sin(theta)
    kx = (r32-r23)/(2*sth)
    ky = (r13-r31)/(2*sth)
    kz = (r21-r12)/(2*sth)

    rv = [theta*kx,theta*ky,theta*kz]
    # rv[0] = theta*kx
    # rv[1] = theta*ky
    # rv[2] = theta*kz
    return rv
   
def rv2rpy(rv):
    rx,ry,rz = rv
    #https://gitlab.com/sdurobotics/ur_rtde/-/issues/69
    theta = sqrt(rx*rx + ry*ry + rz*rz)
    kx = rx/theta
    ky = ry/theta
    kz = rz/theta
    cth = cos(theta)
    sth = sin(theta)
    vth = 1-cos(theta)

    r11 = kx*kx*vth + cth
    r12 = kx*ky*vth - kz*sth
    r13 = kx*kz*vth + ky*sth
    r21 = kx*ky*vth + kz*sth
    r22 = ky*ky*vth + cth
    r23 = ky*kz*vth - kx*sth
    r31 = kx*kz*vth - ky*sth
    r32 = ky*kz*vth + kx*sth
    r33 = kz*kz*vth + cth

    beta = atan2(-r31,sqrt(r11*r11+r21*r21))

    if beta > pi/2:
        beta = pi/2
        alpha = 0
        gamma = atan2(r12,r22)
    elif beta < -pi/2:
        beta = -pi/2
        alpha = 0
        gamma = -atan2(r12,r22)
    else:
        cb = cos(beta)
        alpha = atan2(r21/cb,r11/cb)
        gamma = atan2(r32/cb,r33/cb)

    return np.array([gamma,beta,alpha])

  
def wrap_to_2pi(phase):
    return (phase + np.pi) % (2 * np.pi) - np.pi

def wrap_to_2pi(phase):
    return (phase + np.pi) % (2 * np.pi) - np.pi
 

def interpol(vec,dt,t):
    i = int(t/dt)
    try:
        val = (vec[i+1]-vec[i])*(t%dt)/dt+vec[i]
    except:
        val = vec[-1]
        # print("Sequence Finished")
    return val

def calc_new_position(global_frame_pos, body_frame_pos, rot):
    return global_frame_pos+rot@body_frame_pos
 
# Frame transformation equations
def transformMatrix_from_pose(pose): 
    Po = np.array(pose[0:3]).reshape(3,1) 
    Ro = rv_to_matrix(pose[3:6])
    Gst0 = np.concatenate((np.concatenate((Ro,Po),axis=1),np.array([[0,0,0,1]])),axis=0) 
    return Gst0

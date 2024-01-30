import numpy as np
import sys
import roslib

import time

roslib.load_manifest("ur_kinematics")
from ur_kinematics.ur_kin_py import forward, inverse

def best_sol(sols, q_guess, weights):
    valid_sols = []
    for sol in sols:
        test_sol = np.ones(6)*9999.
        for i in range(6):
            for add_ang in [-2.*np.pi, 0, 2.*np.pi]:
                test_ang = sol[i] + add_ang
                if (abs(test_ang) <= 2.*np.pi and 
                    abs(test_ang - q_guess[i]) < abs(test_sol[i] - q_guess[i])):
                    test_sol[i] = test_ang
        if np.all(test_sol != 9999.):
            valid_sols.append(test_sol)
    if len(valid_sols) == 0:
        return None
    best_sol_ind = np.argmin(np.sum((weights*(valid_sols - np.array(q_guess)))**2,1))
    return valid_sols[best_sol_ind]

def test_q(q):
    x = forward(q)
#     x = np.array([[ 0.00161112 ,-0.99999869,  0. ,         0.09333599],
#  [ 0.        ,  0.        , -1.   ,      -0.23289999],
#  [ 0.99999869 , 0.00161112 ,-0.   ,       0.52248907],
# [ 0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
    bto0 = np.array([[-1,0,0,0],
                     [0,-1,0,0],
                     [0,0,1,0],
                     [0,0,0,1]])
    sixtoee = np.array([[0,-1,0,0],
                     [0,0,-1,0],
                     [1,0,0,0],
                     [0,0,0,1]])
    # print(x)
    sols = inverse(np.array(x), float(q[5]))

    qsol = best_sol(sols, q, [1.]*6)
    a = time.time()
    # print(time.time()-a)
    # print(sols)
    # print(qsol)
    # print('1-------------')
    # print(np.matmul(sixtoee,np.matmul(bto0,x)))
    # print('2-------------')
    # print(np.matmul(np.linalg.inv(sixtoee),np.matmul(np.linalg.inv(bto0),x)))
    # print('3-------------')
    # print(np.matmul(x,np.matmul(np.linalg.inv(sixtoee),np.linalg.inv(bto0))))
    theta = q[5]
    pose = np.array([[-np.cos(theta),np.sin(theta),0],[0,0,-1],[-np.sin(theta),-np.cos(theta),0]])
    print('4-------------')
    print('q',np.around(q,3))
    print('x',np.around(x,3))
    print(np.around(np.matmul(np.linalg.inv(bto0),np.matmul(x,np.linalg.inv(sixtoee))),3))
    print(np.around(pose))
    print('5-------------')
    pose = np.array([[ 1.    ,     -0.     ,     0.     ,    -0.81720001],
                     [ 0.    ,      0.     ,    -1.     ,    -0.23289999],
                     [ 0.     ,     1.      ,   -0.      ,    0.0628    ],
                     [ 0      ,     0       ,    0       ,    1         ]])
    pose =  np.array([[ 0.00161112 ,-0.99999869,  0. ,         0.09333599],
 [ 0.        ,  0.        , -1.   ,      -0.23289999],
 [ 0.99999869 , 0.00161112 ,-0.   ,       0.52248907],
[ 0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
    x2 = np.matmul(np.matmul(bto0,pose),sixtoee)
    x2 = np.around(x2,5)
    # print(x2)
    # print('6-------------')
    # print('q',q)
    a = time.time()
    sols = inverse(np.array(x2), 0.5235987755982989)
    # sols = inverse(np.array(x2),float())

    qsol = best_sol(sols, q, [1.]*6)
    # print(time.time()-a)
    # print(sols)
    # print(qsol)
    # print('7-------------')
    # print(x-x2)
    input()
    # if qsol is None:
    #     qsol = [999.]*6
    # diff = np.sum(np.abs(np.array(qsol) - q))
    # if diff > 0.001:
    #     print np.array(sols)
    #     print 'Best q:', qsol
    #     print 'Actual:', np.array(q)
    #     print 'Diff:  ', q - qsol
    #     print 'Difdiv:', (q - qsol)/np.pi
    #     print i1-3, i2-3, i3-3, i4-3, i5-3, i6-3
    #     if raw_input() == 'q':
    #         sys.exit()

def main():
    np.set_printoptions(precision=3)
    print("Testing multiples of pi/2...")
    for i1 in range(0,5):
        for i2 in range(0,5):
            print(i1, i2)
            for i3 in range(0,5):
                for i4 in range(0,5):
                    for i5 in range(0,5):
                        for i6 in range(0,5):
                            q = np.array([i1*np.pi/2., i2*np.pi/2., i3*np.pi/2., 
                                          i4*np.pi/2., i5*np.pi/2., i6*np.pi/2.])
                            q = np.array([0.0, -0.6108652381980155, -2.094395102393195, -2.530727415391778, -2.7000623958883807e-13, 0.5235987755982989]  )
                            q = np.array([0.0, -0.6108652381980155, -2.094395102393195, -2.530727415391778, -2.7000623958883807e-13, 0.5235987755982989-np.pi+i6*np.pi/6]  )
                            # q = np.array([0, -1.81508889, -1.58820278, -2.09433333, 0, -1.0472324649479727])
                            # q = np.array([0.0,0,0,0,0,0])
                            q = np.array([0.0,-np.pi/2,0,-np.pi/2,0,90*np.pi/180])
                            q = np.array([0*np.pi/180,75*np.pi/180,90*np.pi/180,15*np.pi/180,90*np.pi/180,0])
                            # q = np.array([0.0, -0.6108652381980155, -2.094395102393195, -2.530727415391778, -2.7000623958883807e-13, 0.5235987755982989-np.pi+0*np.pi/180]  )
                            # print(q)
                            # raw_input()
                            test_q(q)
    # print "Testing random configurations..."
    for i in range(10000):
        q = (np.random.rand(6)-.5)*4*np.pi
        test_q(q)
    # print "Done!"

if __name__ == "__main__":
    if False:
        import cProfile
        cProfile.run('main()', 'ik_prof')
    else:
        main()

# importing python headers
import numpy as np
from sympy import *
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D


'''
Function to calculate Forward kinematics
using DH parameters 
'''
def DH_parameters_transformation_matrix(theta):
    
    theta_i = Symbol("theta_i")
    alpha_i = Symbol("alpha_i")
    a_i = Symbol("a_i")
    d_i = Symbol("d_i")

    # defining transformation matrix for dh parameters
    A_i = Matrix([[cos(theta_i), -sin(theta_i)*cos(alpha_i), sin(theta_i)*sin(alpha_i), a_i*cos(theta_i)],
                  [sin(theta_i),  cos(theta_i)*cos(alpha_i), -cos(theta_i)*sin(alpha_i),  a_i*sin(theta_i)],
                  [0, sin(alpha_i) , cos(alpha_i), d_i], 
                  [0, 0, 0, 1]])

    # parameters from the DH table in the report
    # units converted to cm
    alpha =  Array([0,-pi/2,0,-pi/2,pi/2,-pi/2,0])
    d = Array([0.675, 0, 0, 0.670, 0, 0, 0.115])
    a = Array([0, 0.26, 0.68, -0.035, 0, 0 ,0])

    # successive transformations from frame i to i+1
    T_0_1 = A_i.subs(alpha_i,alpha[0]).subs(a_i,a[0]).subs(d_i,d[0]).subs(theta_i,theta[0])
    T_1_2 = A_i.subs(alpha_i,alpha[1]).subs(a_i,a[1]).subs(d_i,d[1]).subs(theta_i,theta[1])
    T_2_3 = A_i.subs(alpha_i,alpha[2]).subs(a_i,a[2]).subs(d_i,d[2]).subs(theta_i, 0)
    T_3_4 = A_i.subs(alpha_i,alpha[3]).subs(a_i,a[3]).subs(d_i,d[3]).subs(theta_i,theta[2])
    T_4_5 = A_i.subs(alpha_i,alpha[4]).subs(a_i,a[4]).subs(d_i,d[4]).subs(theta_i,theta[3])
    T_5_6 = A_i.subs(alpha_i,alpha[5]).subs(a_i,a[5]).subs(d_i,d[5]).subs(theta_i,theta[4])
    T_6_7 = A_i.subs(alpha_i,alpha[6]).subs(a_i,a[6]).subs(d_i,d[6]).subs(theta_i,theta[5])

    # multiplying consecutive matrices to get end effector to base frame transform
    T_0_2 = T_0_1 * T_1_2
    T_0_3 = T_0_2 * T_2_3
    T_0_4 = T_0_3 * T_3_4
    T_0_5 = T_0_4 * T_4_5
    T_0_6 = T_0_5 * T_5_6
    T_0_7 = T_0_6 * T_6_7
    return T_0_1, T_0_2, T_0_3, T_0_4, T_0_5, T_0_6, T_0_7

'''
Function to generate jacobian
'''
def get_jacobian(T_0_1, T_0_2, T_0_3, T_0_4, T_0_5, T_0_6, T_0_7):
     
    z0 = np.array([0,0,1])
    z1 = np.array(T_0_1[:,2][0:3])
    z2 = np.array(T_0_2[:,2][0:3])
    z3 = np.array(T_0_3[:,2][0:3])
    z4 = np.array(T_0_4[:,2][0:3])
    z5 = np.array(T_0_5[:,2][0:3])
    z6 = np.array(T_0_6[:,2][0:3])

    c0 = np.array([0,0,0])
    c1 = np.array(T_0_1[:,3][0:3])
    c2 = np.array(T_0_2[:,3][0:3])
    c3 = np.array(T_0_3[:,3][0:3])
    c4 = np.array(T_0_4[:,3][0:3])
    c5 = np.array(T_0_5[:,3][0:3])
    c6 = np.array(T_0_6[:,3][0:3])
    c7 = np.array(T_0_7[:,3][0:3])
    ## constructng the jacobian by taking cross product
    j1 = np.cross(z0, c7-c0)
    j2 = np.cross(z1, c7-c1)
    j3 = np.cross(z2, c7-c2)
    j4 = np.cross(z3, c7-c3)
    j5 = np.cross(z4, c7-c4)
    j6 = np.cross(z5, c7-c5)
    j7 = np.cross(z6, c7-c6)

    jacobian = np.array([j1,j2,j4,j5,j6,j7])
    jacobian_rotational = np.array([z0,z1,z3,z4,z5,z6])
    jacobian_transpose = np.array([jacobian.T, jacobian_rotational.T])
    final_jacobian = jacobian_transpose.reshape((6,6))
    return final_jacobian


#### Driver code ####
N = 100
time = 5
vel = 0.1

# initialising plot
fig = plt.figure()
final_plot = fig.add_subplot(111, projection='3d')

# displaying transformation matrix
theta = Symbol("theta")
alpha = Symbol("alpha")
a = Symbol("a")
d = Symbol("d")
    
A_i = Matrix([[cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)],
              [sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha),  a*sin(theta)],
              [0, sin(alpha) , cos(alpha), d], 
              [0, 0, 0, 1]])

# defining an array of all initial joint configurations
q = Array([0,pi/2,pi/2,0,0,0])
i = 0
dt = time/N

# calculating successive forward kinematic matrices
T_0_1, T_0_2, T_0_3, T_0_4, T_0_5, T_0_6, T_0_7 = DH_parameters_transformation_matrix(q)    
    
# calculating jacobian
Jacobian = get_jacobian(T_0_1, T_0_2, T_0_3, T_0_4, T_0_5, T_0_6, T_0_7)
    
# calculating jacobian
Jacobian = np.matrix(Jacobian,dtype='float64') # converting data type for inverse
# taking jacobian inverse for inverse kinematics calculations
Jacobian_inverse = np.linalg.pinv(Jacobian)

# get end-effector velocity vector
end_effector_velocity = np.array([vel*np.sin(i*np.pi/180),vel*np.cos(i*np.pi/180),0,0,0,0])
    
# Inverse velocity Kinematics
q_dot = Jacobian_inverse*(end_effector_velocity.reshape(6,1))
    
# performing numerical integration 
q = Array([q[0]+q_dot[0,0]*dt,q[1]+q_dot[1,0]*dt,q[2]+q_dot[2,0]*dt,q[3]+q_dot[3,0]*dt,q[4]+q_dot[4,0]*dt,q[5]+q_dot[5,0]*dt])
    
# plotting end effector position
x = T_0_7[0,3]
y = T_0_7[1,3]
z = T_0_7[2,3]


print(x)
print(y)
print(z)


    

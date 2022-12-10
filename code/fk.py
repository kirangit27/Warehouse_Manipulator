from sympy import Matrix, symbols, cos, sin, simplify, pi, pprint, diff
from numpy import linspace, meshgrid, ones_like
import matplotlib.pyplot as plt
import time
from mpl_toolkits.mplot3d import axes3d, Axes3D

 
q1, q2, q3, q4, q5, q6, q7 = symbols('q1, q2, q3, q4, q5, q6, q7') # Defining the symbolic joint angles
d1, a1, a2, a3, d4, d7 = 0.675 ,0.260, 0.680, -0.035, 0.670, 0.115 #link lengths

# Function to obtain Transformation matrix between consecutive links 
def get_tf(q,d,a,alpha):
    T = Matrix([[cos(q),-sin(q)*cos(alpha),sin(q)*sin(alpha),a*cos(q)], 
        [sin(q),cos(q)*cos(alpha),-cos(q)*sin(alpha),a*sin(q)], [0,sin(alpha),cos(alpha),d], 
        [0,0,0,1]])    
    return T

# D-H table for the Panda robot as given in the question.
T_01 = get_tf(q1,  d1,     0,      0)
T_12 = get_tf(q2,   0,    a1,  -pi/2)
T_23 = get_tf(q3,   0,    a2,      0)
T_34 = get_tf(q4,  d4,    a3,  -pi/2)
T_45 = get_tf(q5,   0,     0,   pi/2)
T_56 = get_tf(q6,   0,     0,  -pi/2)
T_6n = get_tf(q7,  d7,     0,      0)

T_02 = T_01 * T_12
T_03 = T_02 * T_23
T_04 = T_03 * T_34 
T_05 = T_04 * T_45
T_06 = T_05 * T_56
T_0n = T_06 * T_6n


# Function to compute forward kinematics (position) to obtain the end-effector (x,y,z) co-ords wrt base frame
def forward_position_kinematics(q):
    T = T_0n.evalf(subs={q1: q[0], q2: q[1], q3: q[2], q4: q[3], q5: q[4], q6: q[5]})
    return (T[0,3].round(4),T[1,3].round(4),T[2,3].round(4))

if __name__ == '__main__':

    q_joint = Matrix([0, 0, 0, 0, 0, 0]) # initial joint angles of the robot
    #q_joint = Matrix([-1.12, 0.91, 1.33, 0.26, 0.38, 0.0])

    (x, y, z) = forward_position_kinematics(q_joint)
    pprint(T_0n)
    print(x)
    print(y)
    print(x)


    

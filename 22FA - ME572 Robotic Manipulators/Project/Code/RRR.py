from math import sqrt, atan2, acos, sin, cos

def inverseKinematics(px, py, pz):
    L1, L2, L3, D1 = 5, 5, 4, 4
    e = (px*px + py*py)
    th1 = atan2(py, px) - acos(D1 / sqrt(e))
    a = px - (D1 * cos(th1))
    b = py - (D1 * sin(th1))
    c = pz - L1
    d = (px * sin(th1)) - (py*cos(th1))
    th3 = acos((a*a + b*b + c*c - L2*L2 - L3*L3) / (2*L2*L3))
    num = (d * (L2 + L3*cos(th3))) - (c * L3 * sin(th3))
    den = (c * (L2 + L3*cos(th3))) + (d * L3 * sin(th3))
    th2 = atan2(num,den)
    return [th1, th2, th3]

from numpy import matrix
from numpy.linalg import inv

def jacobian(th1, th2, th3, vx, vy, vz, px, py, pz):
    L1, L2, L3, D1 = 5, 5, 4, 4
    th23 = th2 + th3

    j11 = -py
    j12 = (L3 * sin(th1) * cos(th23)) + (L2 * sin(th1) * cos(th2))
    j13 = L3 * sin(th1) * cos(th23)

    j21 = px
    j22 = (-L3 * cos(th1) * cos(th23)) - (L2 * cos(th1) * cos(th2))
    j23 = -L3 * cos(th1) * cos(th23)

    j31 = 0
    j32 = (-L3 * sin(th23)) - (L2 * sin(th2))
    j33 = -L3 * sin(th23)

    J = matrix([[j11, j12, j13],
                [j21, j22, j23],
                [j31, j32, j33]])
    Jinv = inv(J)
    vxyz = matrix([[vx], [vy], [vz]])
    output = Jinv @ vxyz # the "@" is matrix multiplication

    th1d = output[0,0]
    th2d = output[1,0]
    th3d = output[2,0]
    return [th1d, th2d, th3d]
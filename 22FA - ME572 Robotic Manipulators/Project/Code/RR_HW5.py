from math import sqrt, atan2, acos, sin, cos

def inverseKinematics(px, py, pz):
    L1, L2 = 20, 20
    a = (px*px + py*py + L1*L1 - L2*L2) / (2*L1)
    b = sqrt(px*px + py*py)
    th1 = atan2(py, px) + acos(a/b)
    num = py - (L1 * sin(th1))
    den = px - (L1 * cos(th1))
    th2 = atan2(num, den) - th1
    th3 = 0
    return [th1, th2, th3]

from numpy import matrix
from numpy.linalg import inv

def jacobian(th1, th2, th3, vx, vy, vz, px, py, pz):
    L1, L2 = 20, 20
    th12 = th1 + th2

    A = (-L1 * sin(th1))  + (-L2 * sin(th12))
    B = (-L2) * sin(th12)
    C = (L1 * cos(th1)) + (L2 * cos(th12))
    D = L2 * cos(th12)

    J = matrix([[A, B],
                [C, D]])
    Jinv = inv(J)
    vxy = matrix([[vx], [vy]])
    output = Jinv @ vxy # the "@" is matrix multiplication

    th1d = output[0,0]
    th2d = output[1,0]
    th3d = 0
    return [th1d, th2d, th3d]
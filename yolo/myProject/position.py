import numpy as np
from math import *
from mavros_msgs.msg import Altitude, GlobalPositionTarget

def matrix_calcul(n):
    matrix = np.mat(n)
    ans = (np.linalg.det((matrix)))
    print(ans)
    return ans


'''
n = np.array([[1, 3], [1, 5]])
matrix_calcul(n)
'''


class vector():
    def __init__(self):
        self.x = np.double(0.0)
        self.y = np.double(0.0)
        self.z = np.double(0.0)
    

class horizontalTargetPositioning():
    def __init__(self):
        self.firPos = vector()
        self.secPos = vector()
        self.groundTargetPos = vector()
        self.targetHeading = vector()

    def groundTargetPostion(self):
        '''        
        #x = (tanθ_z1 * x_1) - (tanθ_z0 * x_0) - (y_1-y_0)) / (tanθ_z1 - tanθ_z0)
        self.groundTargetPos.x = tan(self.secPos.z)*self.secPos.x \
            - tan(self.firPos.z)*self.firPos.x \
            - (self.secPos.y-self.firPos.y) \
            / tan(self.secPos.z-self.firPos.z)

        #y = ((tanθ_z0 * tanθ_z1)*x_1 - (tanθ_z0*tanθ_z1)*x_0 - [(tanθ_z0)y_1-(tanθ_z1)*y_0]) / (tanθ_z1-tanθ_z0)
        self.groundTargetPos.y = (tan(self.firPos.z)*tan(self.secPos.z)*self.secPos.x) \
            - (tan(self.firPos.z)*tan(self.secPos.z)*self.firPos.x) \
            - (tan(self.firPos.z)*self.secPos.y - tan(self.secPos.z)*self.firPos.y) \
            / tan(self.secPos.z) - tan(self.firPos.z)

        #z = z_0 - (tanθ_y1) * sqrt((x-x_1)^2 + (y-y_1)^2)
        self.groundTargetPos.z = self.firPos.z - tan(self.firPos.y) \
            * sqrt(((self.groundTargetPos.x - self.firPos.x)**2)
                   + (self.groundTargetPos.y - self.firPos.y)**2)
        '''

        self.groundTargetPos.x = (tan(self.secPos.y)*self.secPos.x - tan(self.firPos.y)*self.firPos.x) / tan(self.secPos.y) - tan(self.firPos.y)
        self.groundTargetPos.y = (tan(self.secPos.y)*self.secPos.y - tan(self.firPos.y)*self.firPos.y) / tan(self.secPos.y) - tan(self.firPos.y)
        self.groundTargetPos.z = self.firPos.z - self.firPos.y/cos(self.firPos.z) - self.groundTargetPos.x - self.firPos.x

    def pitch_yaw_degreesAdd(self,dronePitch, droneYaw, motorPitch, motorYaw):
        self.targetHeading.y = dronePitch + motorPitch
        self.targetHeading.z = droneYaw + motorYaw
        return self.targetHeading.y, self.targetHeading.z
    
'''
class verticalTargetPositioning():
    def __init__(self):
        self.firPos = vector()
        self.secPos = vector()
        self.groundTargetPos = vector()
        self.targetAngles = vector()
        self.D_xy = (self.secPos - self.firPos) / (tan())
        
    def groundTargetPostion(self):
        self.groundTargetPos.x = 
        self.groundTargetPos.y = 
        self.groundTargetPos.z = 

'''
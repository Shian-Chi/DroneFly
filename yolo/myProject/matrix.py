import numpy as np
from math import *

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
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class horizontalTargetPositioning():
    def __init__(self):
        self.pos_zero = vector()
        self.pos_one = vector()
        self.groundTargetPos = vector()
        self.targetHeading = vector()

    def groundTargetPostion(self):

        '''        
        #x = (tanθ_z1 * x_1) - (tanθ_z0 * x_0) - (y_1-y_0)) / (tanθ_z1 - tanθ_z0)
        self.groundTargetPos.x = tan(self.pos_one.z)*self.pos_one.x \
            - tan(self.pos_zero.z)*self.pos_zero.x \
            - (self.pos_one.y-self.pos_zero.y) \
            / tan(self.pos_one.z-self.pos_zero.z)

        #y = ((tanθ_z0 * tanθ_z1)*x_1 - (tanθ_z0*tanθ_z1)*x_0 - [(tanθ_z0)y_1-(tanθ_z1)*y_0]) / (tanθ_z1-tanθ_z0)
        self.groundTargetPos.y = (tan(self.pos_zero.z)*tan(self.pos_one.z)*self.pos_one.x) \
            - (tan(self.pos_zero.z)*tan(self.pos_one.z)*self.pos_zero.x) \
            - (tan(self.pos_zero.z)*self.pos_one.y - tan(self.pos_one.z)*self.pos_zero.y) \
            / tan(self.pos_one.z) - tan(self.pos_zero.z)

        #z = z_0 - (tanθ_y1) * sqrt((x-x_1)^2 + (y-y_1)^2)
        self.groundTargetPos.z = self.pos_zero.z - tan(self.pos_zero.y) \
            * sqrt(((self.groundTargetPos.x - self.pos_zero.x)**2)
                   + (self.groundTargetPos.y - self.pos_zero.y)**2)
        '''

        self.groundTargetPos.x = (tan(self.pos_one.y)*self.pos_one.x - tan(self.pos_zero.y)*self.pos_zero.x) / tan(self.pos_one.y) - tan(self.pos_zero.y)
        self.groundTargetPos.y = (tan(self.pos_one.y)*self.pos_one.y - tan(self.pos_zero.y)*self.pos_zero.y) / tan(self.pos_one.y) - tan(self.pos_zero.y)
        self.groundTargetPos.z = self.pos_zero.z - self.pos_zero.y/cos(self.pos_zero.z) - self.groundTargetPos.x - self.pos_zero.x

    def pitch_yaw_degreesAdd(self,dronePitch, droneYaw, motorPitch, motorYaw):
        self.targetHeading.y = dronePitch + motorPitch
        self.targetHeading.z = droneYaw + motorYaw
        return self.targetHeading.y, self.targetHeading.z
    
    
class verticalTargetPositioning():
    def __init__(self):
        self.pos_zero = vector() # X_0, Y_0, Z_0
        self.pos_one = vector() # X_1, Y_1, Z_1
        self.groundTargetPos = vector()
        self.gimbleAngles_zero = vector() # θ_x0, θ_y0, θ_z0
        self.gimbleAngles_one = vector() # θ_x1, θ_y1, θ_z1
        self.D_xy = (self.pos_one.z - self.pos_zero.z) / (tan(self.gimbleAngles_zero.y) - tan(self.gimbleAngles_one.y))
    
    # Calculate target position
    def groundTargetPostion(self): 
        self.groundTargetPos.x = self.pos_zero.x + self.D_xy * cos(self.gimbleAngles_zero.z)
        self.groundTargetPos.y = self.pos_zero.y + self.D_xy * sin(self.gimbleAngles_zero.z)
        self.groundTargetPos.z = self.pos_one.z - self.D_xy * tan(self.pos_one.y)
        return self.groundTargetPos
    
    # input new position
    def newPos(self, x, y, z):
        # last position
        self.pos_zero.x = self.pos_one.x
        self.pos_zero.y = self.pos_one.y
        self.pos_zero.z = self.pos_one.z
        
        # latest position
        self.pos_one.x = x
        self.pos_one.y = y
        self.pos_one.z = z
        
        
    def newgimbleAngles(self, x, y, z):
        # last gimble angles
        self.gimbleAngles_zero.x = self.gimbleAngles_one.x
        self.gimbleAngles_zero.y = self.gimbleAngles_one.y
        self.gimbleAngles_zero.z = self.gimbleAngles_one.z
        
        # latest gimble angles
        self.gimbleAngles_one.x = x
        self.gimbleAngles_one.y = y
        self.gimbleAngles_one.z = z
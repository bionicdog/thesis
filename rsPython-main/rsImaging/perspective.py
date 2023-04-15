# -*- coding: utf-8 -*-
"""
Created on Wed May 27 15:06:29 2020

@author: Jan Lemeire

for rotations:
    https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.html
    https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.apply.html#scipy.spatial.transform.Rotation.apply

"""
import numpy as np
from scipy.spatial.transform import Rotation as R
import math

# scr: image frame; pix: pixel/projection frame
def scr2pix( pix , width, height):
    return (int(pix[0] - width / 2), int(height / 2 - pix[1]) )

def pix2scr( pix , width, height):
    return (int(pix[0] + width / 2), int(height / 2 - pix[1]) )

class Perspective:
    # attitude = pitch (, roll), yaw
    def __init__(self, frho, pos, attitude):
        self.frho = frho
        self.pos = pos
        self.attitude = attitude
        self.r = R.from_euler('xz', attitude, degrees=False)
        
    """ Global <> Image (projection/pixel frame)   """
    def global2image(self, pglobal):
        return self.local2image(self.global2local(pglobal))

    """ Global <> Ray  """
    def global2rayRatios(self, pglobal):
        return self.local2rayRatios(self.global2local(pglobal))
    
    """ Global <> Local (camera frame)  """   
    def global2local(self, v):
        v_trans = v - self.pos
        return self.r.apply(v_trans, inverse = True)
    def rotate2local(self, v):
        return self.r.apply(v, inverse = True)
    
    def local2global(self, local):
        return self.translate2global( self.rotate2global(local))
    def translate2global(self, _local):
        return _local + self.pos
    def rotate2global(self,local):
        return self.r.apply(local)
    
    """ Local <> Image (projection/pixel frame) """
    MINIMAL_Z = 0.1
    def local2rayRatios(self, pcamera):
        if abs (pcamera[2]) < self.MINIMAL_Z:
            pcamera[2] = self.MINIMAL_Z if pcamera[2] <= 0 else -self.MINIMAL_Z
        return (pcamera[0] / pcamera[2], pcamera[1] / pcamera[2])
    def local2image(self, pcamera):
        rayRatios = self.local2rayRatios(pcamera)
#        return ((int) (rayRatios[0] * self.frho), (int)(-rayRatios[1] * self.frho) ) # negate Y!!
        return ( rayRatios[0] * self.frho, -rayRatios[1] * self.frho ) # negate Y!!
            
	
    
    # see corresponding java-code  SHOULD BE CHCKED - IT USES DIFFERENT AXIS SYSTEM!!
    def locatePixelOnXZplane(self, v, yValue=0):
        return self.locateRayOnXZplane(v / self.frho, yValue)
    def locateRayOnXZplane(self, ratios, yValue=0):
        # ray ratios
        yplane_normal = np.array((0, 1, 0))
        yplane_point  = np.array((0, yValue, 0))
        yplane_normal_local = self.rotate2local(yplane_normal)
        yplane_point_local = self.global2local(yplane_point)
        equation_d_constant = yplane_normal_local.dot(yplane_point_local)
        
        denominator = ratios[0] * yplane_normal_local[0] + yplane_normal_local[2] + ratios[1] * yplane_normal_local[1]
        z = equation_d_constant / denominator
        x = ratios[0] * z
        y = ratios[1] * z
        local = np.array((x, y, z) )
        return self.local2global(local)

def l2str(l):
    if type(l) == list or type(l) == tuple or type(l) == np.ndarray:
        l_f = ['%.2f'%k for k in l]
        return '[' + ', '.join(l_f) +']'
    else:
        '%.2f'%l
        
def l2strOfInt(l):
    if type(l) == list or type(l) == tuple or type(l) == np.ndarray:
        l_f = [str(int(k)) for k in l]
        return '[' + ', '.join(l_f) +']'
    else:
        '%.2f'%l
        
if __name__== "__main__":
    if True:
        # parameters camera
        camera_frho = 1434.9
        camera_hoogte = 50 # cm
        camera_pitch = -math.pi/4 # camera kijkt schuin naar beneden onder een hoek van 45 graden
        
        camera_pos = np.array(( 0, -camera_hoogte, 0) ) # Y-as is downwards
        camera_attitude = np.array( (camera_pitch, 0) ) # pitch , yaw
        persp = Perspective(camera_frho, camera_pos, camera_attitude)
        
        # punt in de wereld => pixel in image
        pglobal = np.array( (20, 0, 30) )
        plocal = persp.global2local(pglobal) # camera frame
        pproj = persp.local2image(plocal) # projection frame
        
        image_width = 1000
        image_height = 500

        ppix = scr2pix(pproj, image_width, image_height) # image frame (row, column)
        print('Global '+str(pglobal) + ' => camera '+l2str(plocal)+ ' => pixel '+l2strOfInt(pproj)+' (projection frame) => image '+l2strOfInt(ppix))
    
        # pixel on image => global frame 
        yValue = 0  # we know the height of the object
        p = persp.locatePixelOnXZplane( np.array(pproj) , yValue) # projection/pixel frame
        print('Pixel '+l2str(pproj) + ' results in point '+l2strOfInt(p)+ ' in the XZ-plane (Y='+str(yValue)+')')


    if False:
        alfa = 0.973 # pitch
        beta = 0.041 # yaw
        y = 324
        frho = 1434.9
        pos_camera = np.array(( 0, 0, y) ) 
        attitude_camera = np.array( (-alfa, beta) )

        persp = Perspective(frho, pos_camera, attitude_camera)
        v = np.array( (-384, -29) )  # pixel
        p = persp.locatePixelOnXZplane(v) # projection/pixel frame
        print('Pixel '+str(v) + ' results in point '+l2str(p)+ ' in the Z-plane (Y=0)')
        
        # THIS SEEMS NOT CORRECT
        
        # we check by projecting back
        
        plocal = persp.global2local(p) # camera frame
        pproj = persp.local2image(plocal) # projection frame
        print('Global '+l2str(p) + ' => camera '+l2str(plocal)+ ' => pixel '+l2strOfInt(pproj)+' (projection frame)  ')

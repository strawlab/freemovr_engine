import numpy as np
import math
from scipy import ndimage
import cv2

class sphere_geometry:
    def calculate_distance(self, Ub, Vb, UV_scale, U, V):
        """calculate distances to given polygon on a spherical geometry
        takes poles in UV into account
                
        Args:
            Ub, Vb:     polygon as vector of UV coordinates on the cylinder (U=circumference, V=height)
            UV_scale:   scale of internal lookup table for UV
            U, V:       UV coordinates for which distances are going to be calculated
            
        Returns:
            distances to input U,V coords in the same order as the input
            distances are calculated from the inner contour of this polygon (outside=0)
             
        """

        R = self.__map_UV_to_XYZ(Ub, Vb)
        
        # calculate mean vector of boundary
        v=np.sum(R, axis=1)/R.shape[1]
        # rotate all point so that mean points to +Z
        rotation = self.__rotvec2mat(v, np.array([0,0,1.0]))
        X,Y = self.__map_to_plane_angular(R, rotation, UV_scale)

        img=np.zeros(UV_scale)
        UV=np.array([np.array((Y,X))]).transpose(2,0,1).astype(np.int32)        
        # fill beam path in UV space with 
        cv2.drawContours(img, [UV], 0, 1, -1)
        img=ndimage.distance_transform_edt(img)

        R = self.__map_UV_to_XYZ(U, V) # R contains now XYZ of all pixels with valid UV
        Rp= self.__map_to_plane_angular(R, rotation, UV_scale)
#       Rp contains now all valid pixels in transformed UV coordinates 
        
        u=Rp[0].astype(int)
        v=Rp[1].astype(int)
        return img[(u,v)]

    def __unitsph2cart(self, lon, lat):
        """ Transform coordinates on the unit sphere to Cartesian
        """
        x = np.cos(lat) * np.cos(lon)
        y = np.cos(lat) * np.sin(lon)
        z = np.sin(lat)
        return np.array([x, y, z])
    
    def __map_UV_to_XYZ(self, U, V):
        """ map UV to XYZ on unit sphere 
        """
        azimuth=(U-0.5)*2.0*math.pi;
        elevation=(V-0.5)*math.pi;
        return self.__unitsph2cart(azimuth, elevation)
    
    def __cart2sph(self, x, y, z):
        """ Transform Cartesian coordinates to spherical
        """
        azimuth = np.arctan2(y,x)
        xy2=np.square(x) + np.square(y)
        xy=np.sqrt(xy2)
        elevation = np.arctan2(z,xy)
        radius = np.sqrt(xy2 + np.square(z))
        return (azimuth, elevation, radius)
    
    def __pol2cart(self, theta, rho):
        """Transform polar coordinates to Cartesian
        """
        x = rho * np.cos(theta)
        y = rho * np.sin(theta)
        return x, y
        
    def __map_to_plane_angular(self, x, rotation, UV_scale):
        """ map XYZ on unit sphere to XY plane with rotation & scale
        
         maps one pole of the sphere to [0.5, 0.5]*UV_scale and the other one to the unit
         circle inscribed in the square [0, 0]-[1, 1]*UV_scale
        
         'rotation' normally rotates the 'x' so that [0.5, 0.5]*UV_scale lies
         in the middle of 'x' to avoid singularities in the mappingg of 'x'
        """
        mR=np.matrix(x)
         
        # rotate points so that mean is oriented +Z
        k=np.array(rotation.T*mR)
        (azimuth, elevation, _) = self.__cart2sph(k[0,:], k[1,:], k[2,:])
        X,Y = self.__pol2cart(azimuth,math.pi/2-elevation)
        A=(X/2.0/math.pi+0.5) * UV_scale[0]
        B=(Y/2.0/math.pi+0.5) * UV_scale[1]
        return A,B

    def __rotvec2mat(self, fromVec, toVec):
        """ Calculate rotation matrix defined by rotating fromVec into toVec
        """
        fromVec = fromVec / np.linalg.norm(fromVec)
        toVec = toVec / np.linalg.norm(toVec)
    
        # calculate rotation axis and angle 
        a = np.cross(fromVec, toVec)
        angle = math.acos(np.dot(fromVec, toVec))
        x, y, z = a
        al = np.linalg.norm(a)
        if al != 0.0: # degenerate case
            a = a / al

        sina = math.sin(angle)
        cosa = math.cos(angle)
        rot=np.zeros((3,3))
        # Euler-Rodrigues rotation matrix
        rot[0,0]=1.0+(1.0-cosa)*(x**2-1.0)
        rot[0,1]=z*sina+(1.0-cosa)*x*y
        rot[0,2]=-y*sina+(1.0-cosa)*x*z

        rot[1,0]=-z*sina+(1.0-cosa)*x*y
        rot[1,1]=1.0+(1.0-cosa)*(y**2-1.0)
        rot[1,2]=x*sina+(1.0-cosa)*y*z

        rot[2,0]=y*sina+(1.0-cosa)*x*z
        rot[2,1]=-x*sina+(1.0-cosa)*y*z
        rot[2,2]=1.0+(1.0-cosa)*(z**2-1.0)

        return rot
#!/usr/bin/env python
import pcl
import argparse
import numpy as np
import os
from scipy.optimize import fmin
import flydra.reconstruct

import roslib;
roslib.load_manifest('freemovr_engine')
from calib.reconstruct import PointCloudTransformer
from calib.io import MultiCalSelfCam, save_ascii_matrix
import tf
from freemovr_engine.simple_geom import Cylinder

class Autofitter:
    def __init__(self, filename, orig_flydra_R, out_fname=None):
        self.srcR = flydra.reconstruct.Reconstructor( orig_flydra_R )
        self.orig_XCR_e = MultiCalSelfCam.read_calibration_result( orig_flydra_R )

        self.out_flydra_R = orig_flydra_R + '.aligned'

        print self.out_flydra_R

        assert not os.path.exists(self.out_flydra_R)

        pcd = pcl.PointCloud()
        pcd.from_file( filename )

        self.verts = pcd.to_array()
        height = 1.0
        self.cyl = Cylinder( base=dict(x=0,y=0,z=0),
                             axis=dict(x=0,y=0,z=height),
                             radius=0.5)

        self._centers = np.zeros_like(self.verts)
        self._centers[:,2] = height/2.0

        if out_fname is None:
            base = os.path.splitext(filename)[0]
            self.out_fname = base + '-cyl.pcd'
        else:
            self.out_fname = out_fname
        assert self.out_fname.endswith('.pcd')

    def run(self):
        # make some initial guesses.
        mean_v = np.mean(self.verts,axis=0)
        std_v = np.std(self.verts,axis=0)

        # FIXME: this is hard-coded to a unit cylinder.
        target_center = np.array([0,0,0.5])
        translate = target_center - mean_v
        scale = np.mean(0.5/std_v) # this is hacky...

        quat = tf.transformations.quaternion_from_euler(0,0,0)
        p0 = np.array([scale] + translate.tolist() + quat.tolist())
        print 'p0'
        print p0
        self.orig_scale = scale

        if 1:
            recon0 = self.get_recon_from_param_vec(p0)
            M0 = recon0.get_transformation_matrix()
            recon0.save_as_json('recon0.json')

            if 1:
                # debug xform before opt
                xe, ce, re = self.orig_XCR_e
                MultiCalSelfCam.transform_and_save( xe, ce, re, '/tmp', recon0 )

        start_err = self.calc_error( p0 )
        print 'start_err',start_err

        p01 = np.asfarray(p0)
        p02 = p01.flatten()

        all_out = fmin( self.calc_error, p0, full_output=True)
        popt = all_out[0]

        final_err = self.calc_error( popt )
        print 'final_err',final_err
        print popt
        print repr(popt)

        recon_opt = self.get_recon_from_param_vec(popt)
        if 1:
            recon_opt.save_as_json('recon_opt.json')

        aligned_verts = recon_opt.move_cloud(self.verts)
        transformed_points = pcl.PointCloud()
        transformed_points.from_array(aligned_verts.astype(np.float32))
        transformed_points.to_file( self.out_fname )
        print 'saved transformed points to', self.out_fname

        M_opt = recon_opt.get_transformation_matrix()
        if 1:
            alignedR = self.srcR.get_aligned_copy(M_opt)
            alignedR.save_to_files_in_new_directory(self.out_flydra_R)

            xe, ce, re = self.orig_XCR_e
            MultiCalSelfCam.transform_and_save( xe, ce, re, self.out_flydra_R, recon_opt )


    def get_recon_from_param_vec(self,p):
        s, tx, ty, tz, q0, q1, q2, q3 = p
        t = np.array( [[tx,ty,tz]], dtype=np.float )
        R = tf.transformations.quaternion_matrix( (q0,q1,q2,q3) )[:3,:3]
        recon = PointCloudTransformer(scale=s,
                                      translate=t,
                                      rotate=R)
        return recon

    def calc_error(self, p ):
        """calculate the distance from the unit cylinder of the verts transformed by p"""
        recon = self.get_recon_from_param_vec(p)
        new_cloud = recon.move_cloud(self.verts)

        surf1 = self.cyl.get_first_surface( self._centers, new_cloud )
        valid_cond = ~np.isnan(surf1[:,0])

        valid_surf = surf1[valid_cond]
        valid_new = new_cloud[valid_cond]
        dist_squared = np.sum((valid_surf-valid_new)**2,axis=1)

        n_invalid = np.sum( ~valid_cond )
        invalid_penalty = 1e5

        this_scale = p[0]
        scale_diff = this_scale-self.orig_scale
        scale_penalty = 1e8* scale_diff**4.0

        error = np.mean(dist_squared) + n_invalid*invalid_penalty + scale_penalty
        return error

if __name__=='__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('filename',
                        help='name of .pcd file',
                        )
    parser.add_argument('recon',
                        help='name of flydra reconstructor directory',
                        )
    args = parser.parse_args()
    af = Autofitter(args.filename, args.recon)
    af.run()

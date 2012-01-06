/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: t; c-basic-offset: 4 -*- */
#ifndef CAMERA_MODEL_H
#define CAMERA_MODEL_H

#include <osg/Camera>

class CameraModel {
public:
	CameraModel(unsigned int width, unsigned int height, bool y_up=true);

	// get basic 2D image information
	unsigned int width() const { return _width; }
	unsigned int height() const {return _height; }

	// get extrinsic parameter information
	osg::Vec3 eye() const;// const {return _eye;}
	osg::Vec3 center() const;// const {return _center;}
	osg::Vec3 up() const;// const {return _up;}

	// get matrices
	osg::Matrixd projection(float znear, float zfar) const;
	osg::Matrixd view() const;

	// get viewer geometry
	osg::ref_ptr<osg::Group> make_rendering(float size) const;

	// setters
	//  - extrinsics
	void set_extrinsic( osg::Vec3 eye, osg::Vec3 center, osg::Vec3 up );

	//  - intrinsics (3x3 matrix upper triangular K normalized so K22 is 1.)
	void set_intrinsic( double K00, double K01, double K02,
						double K11, double K12 );

private:
	unsigned int _width;
	unsigned int _height;
	float _K00;
	float _K01;
	float _K02;
	float _K11;
	float _K12;
	bool _y_up;
	osg::Vec3 _eye;
	osg::Vec3 _center;
	osg::Vec3 _up;

	bool intrinsic_valid;
	bool extrinsic_valid;

};

CameraModel* make_real_camera_parameters();
#endif

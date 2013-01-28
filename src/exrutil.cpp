/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
#include "exrutil.h"

#include <osg/Texture2D>

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/FileUtils>

#include <OpenEXR/ImfTestFile.h>
#include <OpenEXR/ImfRgbaFile.h>
#include <OpenEXR/ImfArray.h>
#include <OpenEXR/ImathBox.h>

#include <sstream>

#include <assert.h>

void save_exr( std::string filename, osg::Image* image ) {
	unsigned int proj_width = image->s();
	unsigned int proj_height = image->t();

	osg::Vec4 color;

	Imf::Array2D<Imf::Rgba> pixels;
	pixels.resizeErase(proj_height, proj_width);

	for (unsigned int i=0; i<proj_width; i++) {
		for (unsigned int j=0; j<proj_height; j++) {
			Imf::Rgba &dst_bits = pixels[j][i];
			color = image->getColor(i,j);
			dst_bits.r = color[0];
			dst_bits.g = color[1];
			dst_bits.b = color[2];
			dst_bits.a = color[3];
		}
	}

	Imf::RgbaOutputFile file(filename.c_str(), proj_width, proj_height, Imf::WRITE_RGB);
	file.setFrameBuffer(&pixels[0][0], 1, proj_width);
	file.writePixels(proj_height);
}

osg::ref_ptr<osg::Image> load_exr( std::string p2c_filename, int& width, int& height,
								   double scale_width, double scale_height ) {

	Imf::Array2D<Imf::Rgba> pixels;
	long projector_width, projector_height;

	// Read OpenEXR file.
	{
        if (!Imf::isOpenExrFile(p2c_filename.c_str())) {
			std::stringstream ss;
			ss << "Input file \"" << p2c_filename<< "\" is not an EXR file.";
			throw std::ios_base::failure(ss.str());
        }

        Imf::RgbaInputFile file(p2c_filename.c_str());
        if (!file.isComplete()) {
			std::stringstream ss;
			ss << "Input file \"" << p2c_filename<< "\" is not complete.";
			throw std::ios_base::failure(ss.str());
        }

        if (file.channels() != Imf::WRITE_RGB) {
			std::stringstream ss;
			ss << "cannot load EXR files that are not RGB";
			throw std::ios_base::failure(ss.str());
        }

        Imath::Box2i dw = file.dataWindow();
        projector_width = dw.max.x - dw.min.x + 1;
        projector_height = dw.max.y - dw.min.y + 1;
        //ROS_DEBUG("Opening EXR file of size %ldx%ld",projector_width,projector_height);
        pixels.resizeErase(projector_height, projector_width);
        file.setFrameBuffer (&pixels[0][0] - dw.min.x - dw.min.y * projector_width, 1, projector_width);
        file.readPixels (dw.min.y, dw.max.y);
	}

	unsigned int proj_width = projector_width;
	unsigned int proj_height = projector_height;
	unsigned int proj_depth = 3;

	float* proj_image_rgb = new float[proj_width*proj_height*proj_depth];
	for (size_t j=0; j< proj_height; j++) {
		size_t row_offset = j*proj_width*proj_depth;
		for (size_t i=0; i< proj_width; i++) {
			size_t col_offset = i*proj_depth;
			size_t offset = row_offset + col_offset;
			Imf::Rgba &p = pixels[j][i];
			proj_image_rgb[offset+0] = (float)(p.r)*scale_width;
			proj_image_rgb[offset+1] = (float)(p.g)*scale_height;
			proj_image_rgb[offset+2] = 0.0f; // B
		}
	}

	osg::Image* result = new osg::Image();
	const float* pRadiance = proj_image_rgb;
	const unsigned char* pData = reinterpret_cast<const unsigned char*>(pRadiance);
	unsigned char* pData1 = const_cast<unsigned char*>(pData);

	result->setInternalTextureFormat(GL_RGB32F);
	result->allocateImage(proj_width, proj_height, 1, GL_RGB, GL_FLOAT);
	memcpy( result->data(), pData1, result->getTotalSizeInBytes());
	assert (result->valid() );

	delete[] proj_image_rgb;

	width=proj_width;
	height=proj_height;

	return result;
}

#ifndef FLYVR_DISPLAY_SURFACE_ARBITRARY_GEOMETRY_H
#define FLYVR_DISPLAY_SURFACE_ARBITRARY_GEOMETRY_H
#include <osg/Group>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Array>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <string>
#include <vector>

#include "DisplaySurfaceGeometry.hpp"

namespace flyvr {

struct TriangleIndex {
  TriangleIndex( unsigned int p1, unsigned int p2, unsigned int p3) : _p1(p1), _p2(p2), _p3(p3) {}
  unsigned int _p1;
  unsigned int _p2;
  unsigned int _p3;
};

class DisplaySurfaceArbitraryGeometry : public GeomModel {
public:
  DisplaySurfaceArbitraryGeometry(std::string filename,double precision);

  int texcoord2worldcoord( double u, double v, double& x, double &y, double &z);
  int worldcoord2texcoord( double x, double y, double z, double &u, double &v);
  int get_first_surface( double ax, double ay, double az,
                         double bx, double by, double bz,
                         double &sx, double &sy, double &sz );

  osg::ref_ptr<osg::Geometry> make_geom(bool texcoord_colors=false);


  void addTriangle(unsigned int p1,unsigned int p2,unsigned int p3);
private:
  void traverse(osg::ref_ptr<osg::Node> nd);
  void traverseGeode(osg::ref_ptr<osg::Geode> geode);
  int invert_coord( double in0, double in1, double in2,
                    double& out0, double &out1, double &out2,
                    osg::ref_ptr<osg::Node> in_node,
                    bool return_3d);

  osg::ref_ptr<osg::Vec3Array> _lineseg_starters;

  std::vector<TriangleIndex> _triangle_indices;
  osg::ref_ptr<osg::Geometry> _geom_with_triangles; // original drawable geometry
  osg::ref_ptr<osg::Geode> _geom_with_triangles_node;
  osg::ref_ptr<osg::Geometry> _texcoords_with_triangles; // vertices are actually original texcoords
  osg::ref_ptr<osg::Geode> _texcoords_with_triangles_node;
  double _precision;
  osg::BoundingSphere _bound;
};

}
#endif

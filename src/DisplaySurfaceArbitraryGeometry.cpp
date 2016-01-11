#include "DisplaySurfaceArbitraryGeometry.h"
#include <stdexcept>
#include <limits>
#include <sstream>

#include <osg/TriangleFunctor>
#include <osg/TriangleIndexFunctor>
#include <osg/io_utils>

#include <osgUtil/LineSegmentIntersector>
#include <osgUtil/IntersectionVisitor>

struct CollectTriangleOperator {
  CollectTriangleOperator():_gi(0) {}
  void setDisplaySurfaceArbitraryGeometry(flyvr::DisplaySurfaceArbitraryGeometry* gi) { _gi = gi; }
  flyvr::DisplaySurfaceArbitraryGeometry* _gi;
  inline void operator()(unsigned int p1, unsigned int p2, unsigned int p3) {
    _gi->addTriangle(p1,p2,p3);
  }
};
typedef osg::TriangleIndexFunctor<CollectTriangleOperator> CollectTriangleIndexFunctor;

using namespace flyvr;

DisplaySurfaceArbitraryGeometry::DisplaySurfaceArbitraryGeometry(std::string filename,double precision) : _precision(precision) {
  osg::ref_ptr<osg::Node> loadedModel = osgDB::readNodeFile(filename);
  if (!loadedModel.valid()) {
    std::stringstream ss;
    ss << "error opening file '" << filename << "'";
    throw std::runtime_error(ss.str());
  }

  // fill the variables _triangle_indices and _geom_with_triangles
  traverse( loadedModel );

  // find bounding sphere
  _bound = loadedModel->getBound();

  if (_triangle_indices.size()==0) {
    throw std::runtime_error("No geometry was found.");
  }

  if (!_geom_with_triangles.valid()) {
    throw std::runtime_error("No drawable geometry was found.");
  }

  // ---------------------------------------------------
  // Make geometry with inverted coordinates but same topology.
  _texcoords_with_triangles = new osg::Geometry();
  {
    osg::Vec3Array *inverted_verts = new osg::Vec3Array; // will actually hold texcoords
    osg::Vec2Array *TexCoordVector = (osg::Vec2Array *)_geom_with_triangles->getTexCoordArray(0);

    // Create Vec3Array with same vertex order as original geometry
    for (unsigned int i=0; i<TexCoordVector->size(); ++i) {
      osg::Vec2d tc1 = TexCoordVector->at(i);
      inverted_verts->push_back( osg::Vec3( tc1[0], tc1[1], 0.0) ); // create 3D vertex from (u,v,0)
    }
    _texcoords_with_triangles->setVertexArray( inverted_verts );

    // Now create triangles with these new verts.
    for (unsigned int i=0; i<_triangle_indices.size(); ++i) {
      osg::DrawElementsUInt* tri =
        new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
      TriangleIndex tri_idxs = _triangle_indices.at(i);
      tri->push_back( tri_idxs._p1 );
      tri->push_back( tri_idxs._p2 );
      tri->push_back( tri_idxs._p3 );
      _texcoords_with_triangles->addPrimitiveSet(tri);
    }
  }
  _texcoords_with_triangles_node = new osg::Geode();
  _texcoords_with_triangles_node->addDrawable( _texcoords_with_triangles );

  // ----------------------------------------------------

  _lineseg_starters = new osg::Vec3Array;
  _lineseg_starters->push_back( osg::Vec3(0.0, 0.0, _precision) );
  _lineseg_starters->push_back( osg::Vec3(_precision, 0.0, 0.0) );
}

osg::ref_ptr<osg::Geometry> const DisplaySurfaceArbitraryGeometry::make_geom(bool texcoord_colors) {
  // Make copy of geometry for drawing. Use only triangles.
  osg::ref_ptr<osg::Geometry> this_geom = new osg::Geometry();
  {
    // Now create triangles with these new verts.
    for (unsigned int i=0; i<_triangle_indices.size(); ++i) {
      osg::DrawElementsUInt* tri =
        new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
      TriangleIndex tri_idxs = _triangle_indices.at(i);
      tri->push_back( tri_idxs._p1 );
      tri->push_back( tri_idxs._p2 );
      tri->push_back( tri_idxs._p3 );
      this_geom->addPrimitiveSet(tri);
    }
  }

  // copy the vertices and texcoords
  osg::Vec3Array* verts = new osg::Vec3Array;
  osg::Vec3Array *orig_verts = dynamic_cast<osg::Vec3Array*>(_geom_with_triangles->getVertexArray());
  for (int i=0; i<orig_verts->size(); ++i) {
    verts->push_back( orig_verts->at(i) );
  }

  osg::Vec2Array* tc = new osg::Vec2Array;
  osg::Vec2Array *orig_tcs = (osg::Vec2Array *)_geom_with_triangles->getTexCoordArray(0);
  for (int i=0; i<orig_tcs->size(); ++i) {
    tc->push_back( orig_tcs->at(i) );
  }

  this_geom->setVertexArray(verts);
  this_geom->setTexCoordArray(0,tc);
  // FIXME: add normals?

  if (texcoord_colors) {
    // FIXME: add colors
    this_geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
  } else {
    // FIXME: add colors
    this_geom->setColorBinding(osg::Geometry::BIND_OVERALL);
  }
  return this_geom;
}


void DisplaySurfaceArbitraryGeometry::addTriangle(unsigned int p1,unsigned int p2,unsigned int p3) {
  _triangle_indices.push_back( TriangleIndex(p1,p2,p3) );
}

int DisplaySurfaceArbitraryGeometry::texcoord2worldcoord( double u, double v, double& x, double &y, double &z) {
  return invert_coord( u, v, 0.0,    x, y, z, _texcoords_with_triangles_node, true);
}

int DisplaySurfaceArbitraryGeometry::worldcoord2texcoord( double x, double y, double z, double &u, double &v) {
  double w;
  return invert_coord(  x, y, z,     u, v, w, _geom_with_triangles_node, false);
}

int DisplaySurfaceArbitraryGeometry::get_first_surface( double ax, double ay, double az,
                                                        double bx, double by, double bz,
                                                        double &sx, double &sy, double &sz ) {
  static const int SUCCESS = 0;

  if (isnan(ax) || isnan(bx)) {
    // If these are nan, the result will be, too. Therefore, just
    // shortcircuit the calls to OSG. (Do not bother checking ay, az
    // or by, bz as only in an unexpected situations would their
    // NaN-ness differ from ax or bx. And in such an unexpected
    // situation, this function will still return correct results.)
    sx = sy = sz = std::numeric_limits<double>::quiet_NaN();
    return SUCCESS;
  }

  osg::Vec3 a = osg::Vec3( ax, ay, az );
  osg::Vec3 b0 = osg::Vec3( bx, by, bz );

  // Calculate the maximum distance that the surface could be.
  osg::Vec3 direction = b0-a;

  double max_dist = (_bound.center()-a).length() + _bound.radius();

  // Now calculate the endpoint for surface testing.
  osg::Vec3 b = a + direction*max_dist;

  osg::ref_ptr<osgUtil::LineSegmentIntersector> intersector = new osgUtil::LineSegmentIntersector(a,b);
  osgUtil::IntersectionVisitor iv = osgUtil::IntersectionVisitor(intersector.get());
  _geom_with_triangles_node->accept(iv);

  bool linehit = intersector->containsIntersections();
  if (!linehit) {
    sx = sy = sz = std::numeric_limits<double>::quiet_NaN();
    return SUCCESS;
  }

  osgUtil::LineSegmentIntersector::Intersection intersection = intersector->getFirstIntersection();

  osg::Vec3 s = intersection.getLocalIntersectPoint();
  sx = s[0];
  sy = s[1];
  sz = s[2];
  return SUCCESS;
}


int DisplaySurfaceArbitraryGeometry::invert_coord( double in0, double in1, double in2,
                                       double& out0, double &out1, double &out2,
                                       osg::ref_ptr<osg::Node> in_node,
                                       bool return_3d) {
  static const int SUCCESS = 0;
  static const int ERROR_NOT_3_RATIOS = 1;
  static const int ERROR_INVALID_SOURCE = 2;

  if (isnan(in0) || isnan(in1) || isnan(in2)) {
    out0 = out1 = out2 = std::numeric_limits<double>::quiet_NaN();
    return SUCCESS;
  }

  osg::Vec3 in_vert = osg::Vec3( in0, in1, in2 );


  // compute all potential intersections, remember the best --------------------
  double best_dist2 = std::numeric_limits<double>::infinity();
  osgUtil::LineSegmentIntersector::Intersection tmp;
  osgUtil::LineSegmentIntersector::Intersection& best_intersection = tmp;
  bool found_any = false;

  for (int i=0; i<_lineseg_starters->size(); ++i) {
    osg::Vec3 a = in_vert + _lineseg_starters->at(i);
    osg::Vec3 b = in_vert - _lineseg_starters->at(i);

    osg::ref_ptr<osgUtil::LineSegmentIntersector> intersector = new osgUtil::LineSegmentIntersector(a,b);
    osgUtil::IntersectionVisitor iv = osgUtil::IntersectionVisitor(intersector.get());
    in_node->accept(iv);

    osgUtil::LineSegmentIntersector::Intersections& intersections = intersector->getIntersections();
    for(osgUtil::LineSegmentIntersector::Intersections::iterator itr = intersections.begin(); itr != intersections.end(); ++itr) {
      const osgUtil::LineSegmentIntersector::Intersection& intersection = *itr;
      double this_dist2 = (in_vert - intersection.getLocalIntersectPoint()).length2();
      if (this_dist2 < best_dist2) {
        best_dist2 = this_dist2;
        best_intersection = *itr;
        found_any = true;
      }

    }
  }

  if (!found_any) {
    // No intersection. Give up.
    out0 = out1 = out2 = std::numeric_limits<double>::quiet_NaN();
    return SUCCESS;
  }

  if ((best_intersection.getLocalIntersectPoint() - in_vert).length() > _precision) {
    out0 = out1 = out2 = std::numeric_limits<double>::quiet_NaN();
    return SUCCESS;
  }

  // with best intersection, compute result ------------------------------
  {
      const osgUtil::LineSegmentIntersector::Intersection::IndexList& indices = best_intersection.indexList;
      const osgUtil::LineSegmentIntersector::Intersection::RatioList& ratios = best_intersection.ratioList;
      if (!(indices.size()==3 && ratios.size()==3)) {
        std::cerr << "did not hit a triangle?" << std::endl;
        return ERROR_NOT_3_RATIOS;
      }
      unsigned int i1 = indices[0];
      unsigned int i2 = indices[1];
      unsigned int i3 = indices[2];

      // barycentric coordinates of intersection

      float r1 = ratios[0];
      float r2 = ratios[1];
      float r3 = ratios[2];

      if (return_3d) {
        // We computed best intersection with texture coords, now find vertex coords.
        osg::Vec3Array *verts = dynamic_cast<osg::Vec3Array*>(_geom_with_triangles->getVertexArray());
        if (!verts) {
          std::cerr << "invalid source" << std::endl;
          return ERROR_INVALID_SOURCE;
        }
        osg::Vec3 wc = verts->at(i1)*r1 + \
                       verts->at(i2)*r2 + \
                       verts->at(i3)*r3;
        out0 = wc.x(); out1=wc.y(); out2=wc.z();
      } else {
        // We computed best intersection with vertex coords, now find texture coords.
        osg::Vec2Array *verts = dynamic_cast<osg::Vec2Array*>(_geom_with_triangles->getTexCoordArray(0));
        if (!verts) {
          std::cerr << "invalid source" << std::endl;
          return ERROR_INVALID_SOURCE;
        }
        osg::Vec2 tc = verts->at(i1)*r1 + \
                       verts->at(i2)*r2 + \
                       verts->at(i3)*r3;
        out0 = tc.x(); out1=tc.y(); out2=0.0;
      }
  }

  return SUCCESS;
}

void DisplaySurfaceArbitraryGeometry::traverse(osg::ref_ptr<osg::Node> nd) {

  // based on http://trac.openscenegraph.org/projects/osg//wiki/Support/ProgrammingGuide/AnalysingAScenegraph
  osg::ref_ptr<osg::Geode> geode = dynamic_cast<osg::Geode *> (nd.get());
  if (geode.valid()) { // traverse the geode. If it isnt a geode the dynamic cast gives NULL.
    traverseGeode(geode);
  } else {
    osg::ref_ptr<osg::Group> gp = dynamic_cast<osg::Group *> (nd.get());
    if (gp.valid()) {
      for (unsigned int ic=0; ic<gp->getNumChildren(); ic++) {
        traverse(gp->getChild(ic));
      }
    } else {
      throw std::runtime_error("not implemented: support for this node type");
    }
  }
}

void DisplaySurfaceArbitraryGeometry::traverseGeode(osg::ref_ptr<osg::Geode> geode) {

  // based on http://trac.openscenegraph.org/projects/osg//wiki/Support/ProgrammingGuide/AnalysingAScenegraph

  for (unsigned int i=0; i<geode->getNumDrawables(); i++) {
    osg::ref_ptr<osg::Drawable> drawable=geode->getDrawable(i);

    osg::ref_ptr<osg::Geometry> geom=dynamic_cast<osg::Geometry *> (drawable.get());
    if (!geom.valid()) {
      throw std::runtime_error("not implemented: support for this drawable");
    }

    if (geom->getNumTexCoordArrays()!=1) {
      throw std::runtime_error("Need exactly one texture coordinate array. Does your model have a texture?");
    }

    CollectTriangleIndexFunctor collectTriangles;
    collectTriangles.setDisplaySurfaceArbitraryGeometry(this);

    unsigned int prev_length = _triangle_indices.size();
    drawable->accept(collectTriangles);
    if (prev_length != 0) {
      if (prev_length != _triangle_indices.size()) {
        throw std::runtime_error("This model has more than one drawable with triangles");
      }
    }

    if (_triangle_indices.size() != 0) {
      _geom_with_triangles = geom;
      _geom_with_triangles_node = geode;
    }

  }
}

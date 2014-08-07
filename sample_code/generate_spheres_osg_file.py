WORLD_TEMPLATE = """
Group {
  UniqueID Group_0
  nodeMask 0xffffffff
  cullingActive TRUE
  num_children 1
  MatrixTransform {
    UniqueID MatrixTransform_1
    nodeMask 0xffffffff
    cullingActive TRUE
    description "root node"
    StateSet {
      UniqueID StateSet_2
      rendering_hint DEFAULT_BIN
      renderBinMode INHERIT
      GL_LIGHTING OFF
    }
    referenceFrame RELATIVE
    Matrix {
      1 0 0 0
      0 1 0 0
      0 0 1 0
      0 0 0 1
    }
    num_children %(num_children)d
    %(children)s
  }
}
"""

SPHERE_TEMPLATE = """
    MatrixTransform {
      name "%(name)s"
      nodeMask 0xffffffff
      cullingActive TRUE
      referenceFrame RELATIVE
      Matrix {
        1 0 0 0
        0 1 0 0
        0 0 1 0
        %(x)f %(y)f %(z)f 1
      }
      num_children 1
      Geode {
        nodeMask 0xffffffff
        cullingActive TRUE
        num_drawables 1
        ShapeDrawable {
          UniqueID ShapeDrawable_%(name)s
          Sphere {
            Center 0 0 0
            Radius %(radius)f
          }
          useDisplayList TRUE
          useVertexBufferObjects FALSE
          color %(r)f %(g)f %(b)f %(a)f
        }
      }
    }
"""

def generate_sphere(name,radius,x,y,z,r,g,b,a):
    return SPHERE_TEMPLATE % dict(
                name=name,
                radius=radius,x=x,y=y,z=z,
                r=r,g=g,b=b,a=a)

if __name__ == "__main__":
    spheres = []
    spheres.append( generate_sphere("s1",0.1,0,0,0,1,0,0,1) )
    spheres.append( generate_sphere("s2",0.2,0,0,0,0,1,0,1) )
    spheres.append( generate_sphere("s3",0.3,0,0,0,0,0,1,1) )

    sphere_txt = "\n".join(spheres)

    osg_txt = WORLD_TEMPLATE % dict(num_children=len(spheres),
                                    children=sphere_txt)

    print osg_txt


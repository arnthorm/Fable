
import array
import cv
import numpy
import math

def get_joint_angles_from_msg(message):
  """..."""
  from user_tracker.msg import JointAng, ArmAng, LegAng
  from os.path import join as join_path
  def crawl(data):
    attr_path = []
    for attr in dir(data):
      if not attr.startswith('_'):
        obj = getattr(data, attr)
        if isinstance(obj, JointAng):
          attr_path.append(attr)
        elif isinstance(obj, ArmAng) or isinstance(obj, LegAng):
          l = crawl(obj)
          for item in l:
            attr_path.append(join_path(attr, item))
    return attr_path
  joints = crawl(message)
  return joints

def get_attr(obj, attrs):
  """
  Returns the value of the attributes of obj given in attrs.
  
  Keyword arguments:
    obj -- object with attribue(s)
    attrs -- the attributes to fetch, can be in the form a/b or a.b,
             where a is attribute of obj and b is attribute of a
  """
  if '/' in attrs:
    attrs = attrs.split('/')
  elif '.' in attrs:
    attrs = attrs.split('.')
  elif type(attrs) not in (list, tuple):
    attrs = (attrs,)
  for attr in attrs:
    if attr:
      obj = getattr(obj, attr)
  return obj


def mkmat(rows, cols, L):
  mat = cv.CreateMat(rows, cols, cv.CV_64FC1)
  cv.SetData(mat, array.array('d', L), 8 * cols)
  return mat


nd_tf = numpy.ndarray((4,4)) 
nd_tf[0] = [  1.0,   0.0,   0.0,  -2.50000000e-02]
nd_tf[1] = [  0.0,   1.0,   0.0,  -3.46944695e-18]
nd_tf[2] = [  0.0,   0.0,   1.0,   0.0]
nd_tf[3] = [  0.0,   0.0,   0.0,   1.0]

P = mkmat(3, 4, [525.0, 0.0, 319.5, 0.0, 0.0, 525.0, 239.5, 0.0, 0.0, 0.0, 1.0, 0.0])

def transform_3d_to_2d(position):
  """
  Transforms 3D position to 2D (image).

  This function is specific for the Asus Xtion Pro Live.
  Will not work with Kinect as the cameras are probably 
  at at different positions. 
  """
  mat44 = nd_tf
  point = tuple(
    numpy.dot(
      mat44, 
      numpy.array([position[0], position[1], position[2], 1.0])
    )
  )[:3]

  src = mkmat(4, 1, [point[0], point[1], point[2], 1.0])
  dst = cv.CreateMat(3, 1, cv.CV_64FC1)
  cv.MatMul(P, src, dst)
  x = dst[0,0]
  y = dst[1,0]
  w = dst[2,0]
  if w != 0:
    return (round(x/w), round(y/w))
  else:
    return (0.0, 0.0)

def get_rect_from_point(point, width, height):
  """
  Get 2D rectangle from 3D point.

  Keyword arguments:
    point  -- Point in 3D coordinates.
    width  -- Width for rectangle in 3D space.
    height -- Height for rectangle in 3D space.
  """
  center3d = get_point(point)
  rect3d = list(center3d)
  rect3d[0] -= width
  rect3d[1] -= height

  center2d = transform_3d_to_2d(center3d)
  rect2d = transform_3d_to_2d(rect3d)
  width = (center2d[0] - rect2d[0])*2
  height = (center2d[1] - rect2d[1])*2
  return (rect2d[0], rect2d[1], width, height)

def get_point(point):
  if hasattr(point, 'x') and hasattr(point, 'y') and hasattr(point, 'z'):
    return [point.x, point.y, point.z]
  elif type(point) is dict and point.has_key('x') and point.has_key('y') and \
      point.has_key('z'):
    return [point['x'], point['y'], point['z']]
  elif type(point) in (list, tuple) and len(point) == 3:
    return point
  else:
    raise ValueError("Value not a point!");

def calc_distance(point):
  point = get_point(point)
  return math.sqrt(point[0]**2 + point[1]**2 + point[2]**2)



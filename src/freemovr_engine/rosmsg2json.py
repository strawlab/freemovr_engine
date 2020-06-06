import json
import base64
import re
import os
import warnings

import roslib.packages
roslib.load_manifest('freemovr_engine')
import freemovr_engine.msg
import geometry_msgs.msg
import std_msgs.msg

ancient_ros = False
try:
    import genpy
except ImportError:
    ancient_ros = True

import roslib.message

import numpy as np

import freemovr_engine.fixup_path as fixup_path

if not ancient_ros:
    time_base_type = genpy.rostime.Time
else:
    time_base_type = roslib.rostime.Time

# globals
re_array = re.compile(r'(.+)\[(\d*)\]$')

BASIC_TYPES = ['float64','float32','string','uint32','uint8','bool','int8','int32']

def _suggestrepl(matchobj):
    ros_pkg_name = matchobj.group(1)
    return 'package://%s' % ros_pkg_name

def convert_attrs( v ):
    return dict( (attr,getattr(v,attr)) for attr in v.__slots__)

def rosmsg2dict(msg):
    plain_dict = {}
    if isinstance(msg, time_base_type):
        plain_dict['secs']=msg.secs
        plain_dict['nsecs']=msg.nsecs
    else:
        assert isinstance(msg, roslib.message.Message)
        for varname, vartype in zip(msg.__slots__, msg._slot_types):

            matchobj = re_array.search(vartype)

            if vartype in BASIC_TYPES:
                plain_dict[varname] = getattr(msg,varname)
            elif vartype == 'Header':
                h = getattr(msg,varname)
                plain_dict[varname] = {'seq':h.seq,
                                       'stamp':rosmsg2dict(h.stamp),# {'secs':h.stamp.secs,
                                                # 'nsecs':h.stamp.nsecs},
                                       'frame_id':h.frame_id}
            elif vartype == 'geometry_msgs/Pose':
                h = getattr(msg,varname)
                plain_dict[varname] = {'position':rosmsg2dict(h.position),
                                       'orientation':rosmsg2dict(h.orientation)}
            elif vartype == 'uint8[]':
                # since ROS message keys have to be valid Python identifiers, we can put metadata here
                plain_dict[varname + ' (base64)'] = base64.b64encode( getattr(msg,varname))
            elif matchobj and matchobj.group(1) in BASIC_TYPES:
                # this must follow 'uint8[]' special case above
                plain_dict[varname] = getattr(msg,varname)
            elif vartype == 'time':
                plain_dict[varname] = rosmsg2dict(getattr(msg,varname))
            elif vartype == 'freemovr_engine/ROSPath':
                plain_dict[varname] = fixup_path.fixup_path(getattr(msg,varname).data)
            elif vartype in ['geometry_msgs/Point',
                             'geometry_msgs/Quaternion',
                             'geometry_msgs/Vector3',
                             'sensor_msgs/RegionOfInterest',
                             'std_msgs/ColorRGBA',
                             ]:
                v = getattr(msg,varname)
                plain_dict[varname] = convert_attrs(v)
            else:
                v = getattr(msg,varname)
                if isinstance(v, roslib.message.Message):
                    # cannot determine type, try recursive conversion
                    plain_dict[varname] = rosmsg2dict(v)
                else:
                    raise ValueError('unknown msg slot type: %s'%vartype)
    return plain_dict

def fixup_keyname(key_with_meta):
    return key_with_meta.split()[0]

def fixup_value( orig_value, keyname):
    fixed_key = fixup_keyname(keyname)
    assert len(fixed_key)>0
    assert keyname.startswith(fixed_key)
    meta = keyname[ len(fixed_key): ]
    if meta == ' (base64)':
        value = base64.b64decode( orig_value )
        return value
    raise ValueError('unknown fixup required')

def is_equal(ros_msg, dict_msg):
    ros_keys = set(ros_msg.__slots__)
    dict_keys_real = list(dict_msg.keys())
    dict_keys = set([fixup_keyname(k) for k in dict_keys_real])
    a = ros_keys-dict_keys
    b = dict_keys-ros_keys
    if not len(a)==0: return False
    if not len(b)==0: return False
    for k in dict_keys_real:
        v_ros = getattr(ros_msg,fixup_keyname(k))
        v_dict = dict_msg[k]
        if k != fixup_keyname(k):
            v_dict = fixup_value( dict_msg[k], k )
        if not v_ros == v_dict:
            if isinstance(v_ros, roslib.message.Message):
                tmp = is_equal(v_ros,v_dict)
                if not tmp:
                    return False
                else:
                    continue
            if isinstance(v_ros, time_base_type):
                if not (v_ros.secs == v_dict['secs'] and v_ros.nsecs == v_dict['nsecs']):
                    return False
                else:
                    continue
            # not equal (could be nan)
            try:
                if np.isnan(v_ros):
                    if np.isnan(v_dict):
                        continue # OK, go to next key
                    else:
                        return False
            except TypeError:
                # could not check if value was nan
                pass
            return False
    return True

def rosmsg2json( msg ):
    plain_dict = rosmsg2dict(msg)
    msg_json = json.dumps( plain_dict, allow_nan=False )
    return msg_json

def compare(msg):
    msg_json = rosmsg2json(msg)
    msg2 = json.loads(msg_json)
    assert is_equal(msg,msg2)

def test_point():
    msg = geometry_msgs.msg.Point()
    msg.x = 1.23456
    msg.y = 2.0
    msg.z = 3.0
    compare(msg)

def test_image():
    msg = freemovr_engine.msg.FreemoVRCompressedImage()
    msg.format = '.png'
    # Smallest PNG image. ( See http://garethrees.org/2007/11/14/pngcrush/ )
    msg.data = ('\x89PNG\r\n\x1a\n\x00\x00\x00\rIHDR\x00\x00\x00\x01\x00\x00\x00\x01\x08\x06'
                '\x00\x00\x00\x1f\x15\xc4\x89\x00\x00\x00\nIDATx\x9cc\x00\x01\x00\x00\x05\x00'
                '\x01\r\n-\xb4\x00\x00\x00\x00IEND\xaeB`\x82')
    compare(msg)

def test_header():
    msg = std_msgs.msg.Header()
    msg.seq = 1234
    msg.frame_id = 'frame0'
    msg.stamp.secs = 123
    msg.stamp.nsecs = 456
    compare(msg)

if __name__=='__main__':
    test_point()
    test_image()
    test_header()

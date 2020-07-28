# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from rail_pose_estimation_msgs/Keypoints.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class Keypoints(genpy.Message):
  _md5sum = "3d1804a99352b413ee0c9ca364640114"
  _type = "rail_pose_estimation_msgs/Keypoints"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """float32 neck_x                  # x coord of neck
float32 neck_y                  # y coord of neck
float32 nose_x                  # x coord of nose
float32 nose_y                  # y coord of nose
float32 right_shoulder_x        # x coord of right shoulder
float32 right_shoulder_y        # y coord of right shoulder
float32 left_shoulder_x         # x coord of left shoulder
float32 left_shoulder_y         # y coord of left shoulder
float32 right_elbow_x           # x coord of right elbow
float32 right_elbow_y           # y coord of right elbow
float32 left_elbow_x            # x coord of left elbow
float32 left_elbow_y            # y coord of left elbow
float32 right_wrist_x           # x coord of right wrist
float32 right_wrist_y           # y coord of right wrist
float32 left_wrist_x            # x coord of left wrist
float32 left_wrist_y            # y coord of left wrist
float32 right_hip_x             # x coord of right hip
float32 right_hip_y             # y coord of right hip
float32 left_hip_x              # x coord of left hip
float32 left_hip_y              # y coord of left hip
float32 right_knee_x            # x coord of right knee
float32 right_knee_y            # y coord of right knee
float32 left_knee_x             # x coord of left knee
float32 left_knee_y             # y coord of left knee
float32 right_ankle_x           # x coord of right ankle
float32 right_ankle_y           # y coord of right ankle
float32 left_ankle_x            # x coord of left ankle
float32 left_ankle_y            # y coord of left ankle
float32 right_eye_x             # x coord of right eye
float32 right_eye_y             # y coord of right eye
float32 left_eye_x              # x coord of left eye
float32 left_eye_y              # y coord of left eye
float32 right_ear_x             # x coord of right ear
float32 right_ear_y             # y coord of right ear
float32 left_ear_x              # x coord of left ear
float32 left_ear_y              # y coord of left ear"""
  __slots__ = ['neck_x','neck_y','nose_x','nose_y','right_shoulder_x','right_shoulder_y','left_shoulder_x','left_shoulder_y','right_elbow_x','right_elbow_y','left_elbow_x','left_elbow_y','right_wrist_x','right_wrist_y','left_wrist_x','left_wrist_y','right_hip_x','right_hip_y','left_hip_x','left_hip_y','right_knee_x','right_knee_y','left_knee_x','left_knee_y','right_ankle_x','right_ankle_y','left_ankle_x','left_ankle_y','right_eye_x','right_eye_y','left_eye_x','left_eye_y','right_ear_x','right_ear_y','left_ear_x','left_ear_y']
  _slot_types = ['float32','float32','float32','float32','float32','float32','float32','float32','float32','float32','float32','float32','float32','float32','float32','float32','float32','float32','float32','float32','float32','float32','float32','float32','float32','float32','float32','float32','float32','float32','float32','float32','float32','float32','float32','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       neck_x,neck_y,nose_x,nose_y,right_shoulder_x,right_shoulder_y,left_shoulder_x,left_shoulder_y,right_elbow_x,right_elbow_y,left_elbow_x,left_elbow_y,right_wrist_x,right_wrist_y,left_wrist_x,left_wrist_y,right_hip_x,right_hip_y,left_hip_x,left_hip_y,right_knee_x,right_knee_y,left_knee_x,left_knee_y,right_ankle_x,right_ankle_y,left_ankle_x,left_ankle_y,right_eye_x,right_eye_y,left_eye_x,left_eye_y,right_ear_x,right_ear_y,left_ear_x,left_ear_y

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Keypoints, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.neck_x is None:
        self.neck_x = 0.
      if self.neck_y is None:
        self.neck_y = 0.
      if self.nose_x is None:
        self.nose_x = 0.
      if self.nose_y is None:
        self.nose_y = 0.
      if self.right_shoulder_x is None:
        self.right_shoulder_x = 0.
      if self.right_shoulder_y is None:
        self.right_shoulder_y = 0.
      if self.left_shoulder_x is None:
        self.left_shoulder_x = 0.
      if self.left_shoulder_y is None:
        self.left_shoulder_y = 0.
      if self.right_elbow_x is None:
        self.right_elbow_x = 0.
      if self.right_elbow_y is None:
        self.right_elbow_y = 0.
      if self.left_elbow_x is None:
        self.left_elbow_x = 0.
      if self.left_elbow_y is None:
        self.left_elbow_y = 0.
      if self.right_wrist_x is None:
        self.right_wrist_x = 0.
      if self.right_wrist_y is None:
        self.right_wrist_y = 0.
      if self.left_wrist_x is None:
        self.left_wrist_x = 0.
      if self.left_wrist_y is None:
        self.left_wrist_y = 0.
      if self.right_hip_x is None:
        self.right_hip_x = 0.
      if self.right_hip_y is None:
        self.right_hip_y = 0.
      if self.left_hip_x is None:
        self.left_hip_x = 0.
      if self.left_hip_y is None:
        self.left_hip_y = 0.
      if self.right_knee_x is None:
        self.right_knee_x = 0.
      if self.right_knee_y is None:
        self.right_knee_y = 0.
      if self.left_knee_x is None:
        self.left_knee_x = 0.
      if self.left_knee_y is None:
        self.left_knee_y = 0.
      if self.right_ankle_x is None:
        self.right_ankle_x = 0.
      if self.right_ankle_y is None:
        self.right_ankle_y = 0.
      if self.left_ankle_x is None:
        self.left_ankle_x = 0.
      if self.left_ankle_y is None:
        self.left_ankle_y = 0.
      if self.right_eye_x is None:
        self.right_eye_x = 0.
      if self.right_eye_y is None:
        self.right_eye_y = 0.
      if self.left_eye_x is None:
        self.left_eye_x = 0.
      if self.left_eye_y is None:
        self.left_eye_y = 0.
      if self.right_ear_x is None:
        self.right_ear_x = 0.
      if self.right_ear_y is None:
        self.right_ear_y = 0.
      if self.left_ear_x is None:
        self.left_ear_x = 0.
      if self.left_ear_y is None:
        self.left_ear_y = 0.
    else:
      self.neck_x = 0.
      self.neck_y = 0.
      self.nose_x = 0.
      self.nose_y = 0.
      self.right_shoulder_x = 0.
      self.right_shoulder_y = 0.
      self.left_shoulder_x = 0.
      self.left_shoulder_y = 0.
      self.right_elbow_x = 0.
      self.right_elbow_y = 0.
      self.left_elbow_x = 0.
      self.left_elbow_y = 0.
      self.right_wrist_x = 0.
      self.right_wrist_y = 0.
      self.left_wrist_x = 0.
      self.left_wrist_y = 0.
      self.right_hip_x = 0.
      self.right_hip_y = 0.
      self.left_hip_x = 0.
      self.left_hip_y = 0.
      self.right_knee_x = 0.
      self.right_knee_y = 0.
      self.left_knee_x = 0.
      self.left_knee_y = 0.
      self.right_ankle_x = 0.
      self.right_ankle_y = 0.
      self.left_ankle_x = 0.
      self.left_ankle_y = 0.
      self.right_eye_x = 0.
      self.right_eye_y = 0.
      self.left_eye_x = 0.
      self.left_eye_y = 0.
      self.right_ear_x = 0.
      self.right_ear_y = 0.
      self.left_ear_x = 0.
      self.left_ear_y = 0.

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_36f().pack(_x.neck_x, _x.neck_y, _x.nose_x, _x.nose_y, _x.right_shoulder_x, _x.right_shoulder_y, _x.left_shoulder_x, _x.left_shoulder_y, _x.right_elbow_x, _x.right_elbow_y, _x.left_elbow_x, _x.left_elbow_y, _x.right_wrist_x, _x.right_wrist_y, _x.left_wrist_x, _x.left_wrist_y, _x.right_hip_x, _x.right_hip_y, _x.left_hip_x, _x.left_hip_y, _x.right_knee_x, _x.right_knee_y, _x.left_knee_x, _x.left_knee_y, _x.right_ankle_x, _x.right_ankle_y, _x.left_ankle_x, _x.left_ankle_y, _x.right_eye_x, _x.right_eye_y, _x.left_eye_x, _x.left_eye_y, _x.right_ear_x, _x.right_ear_y, _x.left_ear_x, _x.left_ear_y))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 144
      (_x.neck_x, _x.neck_y, _x.nose_x, _x.nose_y, _x.right_shoulder_x, _x.right_shoulder_y, _x.left_shoulder_x, _x.left_shoulder_y, _x.right_elbow_x, _x.right_elbow_y, _x.left_elbow_x, _x.left_elbow_y, _x.right_wrist_x, _x.right_wrist_y, _x.left_wrist_x, _x.left_wrist_y, _x.right_hip_x, _x.right_hip_y, _x.left_hip_x, _x.left_hip_y, _x.right_knee_x, _x.right_knee_y, _x.left_knee_x, _x.left_knee_y, _x.right_ankle_x, _x.right_ankle_y, _x.left_ankle_x, _x.left_ankle_y, _x.right_eye_x, _x.right_eye_y, _x.left_eye_x, _x.left_eye_y, _x.right_ear_x, _x.right_ear_y, _x.left_ear_x, _x.left_ear_y,) = _get_struct_36f().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_36f().pack(_x.neck_x, _x.neck_y, _x.nose_x, _x.nose_y, _x.right_shoulder_x, _x.right_shoulder_y, _x.left_shoulder_x, _x.left_shoulder_y, _x.right_elbow_x, _x.right_elbow_y, _x.left_elbow_x, _x.left_elbow_y, _x.right_wrist_x, _x.right_wrist_y, _x.left_wrist_x, _x.left_wrist_y, _x.right_hip_x, _x.right_hip_y, _x.left_hip_x, _x.left_hip_y, _x.right_knee_x, _x.right_knee_y, _x.left_knee_x, _x.left_knee_y, _x.right_ankle_x, _x.right_ankle_y, _x.left_ankle_x, _x.left_ankle_y, _x.right_eye_x, _x.right_eye_y, _x.left_eye_x, _x.left_eye_y, _x.right_ear_x, _x.right_ear_y, _x.left_ear_x, _x.left_ear_y))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 144
      (_x.neck_x, _x.neck_y, _x.nose_x, _x.nose_y, _x.right_shoulder_x, _x.right_shoulder_y, _x.left_shoulder_x, _x.left_shoulder_y, _x.right_elbow_x, _x.right_elbow_y, _x.left_elbow_x, _x.left_elbow_y, _x.right_wrist_x, _x.right_wrist_y, _x.left_wrist_x, _x.left_wrist_y, _x.right_hip_x, _x.right_hip_y, _x.left_hip_x, _x.left_hip_y, _x.right_knee_x, _x.right_knee_y, _x.left_knee_x, _x.left_knee_y, _x.right_ankle_x, _x.right_ankle_y, _x.left_ankle_x, _x.left_ankle_y, _x.right_eye_x, _x.right_eye_y, _x.left_eye_x, _x.left_eye_y, _x.right_ear_x, _x.right_ear_y, _x.left_ear_x, _x.left_ear_y,) = _get_struct_36f().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_36f = None
def _get_struct_36f():
    global _struct_36f
    if _struct_36f is None:
        _struct_36f = struct.Struct("<36f")
    return _struct_36f

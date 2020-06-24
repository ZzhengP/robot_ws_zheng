// Auto-generated. Do not edit!

// (in-package openni2_tracker.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class TrackerUser {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.uid = null;
      this.tracker_id = null;
      this.frames = null;
      this.confs = null;
      this.transforms = null;
      this.projective = null;
      this.center_of_mass = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('uid')) {
        this.uid = initObj.uid
      }
      else {
        this.uid = 0;
      }
      if (initObj.hasOwnProperty('tracker_id')) {
        this.tracker_id = initObj.tracker_id
      }
      else {
        this.tracker_id = '';
      }
      if (initObj.hasOwnProperty('frames')) {
        this.frames = initObj.frames
      }
      else {
        this.frames = [];
      }
      if (initObj.hasOwnProperty('confs')) {
        this.confs = initObj.confs
      }
      else {
        this.confs = [];
      }
      if (initObj.hasOwnProperty('transforms')) {
        this.transforms = initObj.transforms
      }
      else {
        this.transforms = [];
      }
      if (initObj.hasOwnProperty('projective')) {
        this.projective = initObj.projective
      }
      else {
        this.projective = [];
      }
      if (initObj.hasOwnProperty('center_of_mass')) {
        this.center_of_mass = initObj.center_of_mass
      }
      else {
        this.center_of_mass = new geometry_msgs.msg.Vector3();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TrackerUser
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [uid]
    bufferOffset = _serializer.uint8(obj.uid, buffer, bufferOffset);
    // Serialize message field [tracker_id]
    bufferOffset = _serializer.string(obj.tracker_id, buffer, bufferOffset);
    // Serialize message field [frames]
    bufferOffset = _arraySerializer.string(obj.frames, buffer, bufferOffset, null);
    // Serialize message field [confs]
    bufferOffset = _arraySerializer.float64(obj.confs, buffer, bufferOffset, null);
    // Serialize message field [transforms]
    // Serialize the length for message field [transforms]
    bufferOffset = _serializer.uint32(obj.transforms.length, buffer, bufferOffset);
    obj.transforms.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Transform.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [projective]
    // Serialize the length for message field [projective]
    bufferOffset = _serializer.uint32(obj.projective.length, buffer, bufferOffset);
    obj.projective.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Vector3.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [center_of_mass]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.center_of_mass, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TrackerUser
    let len;
    let data = new TrackerUser(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [uid]
    data.uid = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [tracker_id]
    data.tracker_id = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [frames]
    data.frames = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [confs]
    data.confs = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [transforms]
    // Deserialize array length for message field [transforms]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.transforms = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.transforms[i] = geometry_msgs.msg.Transform.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [projective]
    // Deserialize array length for message field [projective]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.projective = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.projective[i] = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [center_of_mass]
    data.center_of_mass = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.tracker_id.length;
    object.frames.forEach((val) => {
      length += 4 + val.length;
    });
    length += 8 * object.confs.length;
    length += 56 * object.transforms.length;
    length += 24 * object.projective.length;
    return length + 45;
  }

  static datatype() {
    // Returns string type for a message object
    return 'openni2_tracker/TrackerUser';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c4f1e9a80fc94f2519473798dffa838a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # TrackerUser: Contains a snapshot of a single users tracking data. an ID, a vector of frames corresponding to each joint, a vector of confidences corresponding to each joint, and a vector of geometry_msgs/Transform messages corresponding to each joint. 
    Header header
    uint8 uid
    string tracker_id
    string[] frames
    float64[] confs
    geometry_msgs/Transform[] transforms
    geometry_msgs/Vector3[] projective
    geometry_msgs/Vector3 center_of_mass
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: geometry_msgs/Transform
    # This represents the transform between two coordinate frames in free space.
    
    Vector3 translation
    Quaternion rotation
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
    float64 x
    float64 y
    float64 z
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TrackerUser(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.uid !== undefined) {
      resolved.uid = msg.uid;
    }
    else {
      resolved.uid = 0
    }

    if (msg.tracker_id !== undefined) {
      resolved.tracker_id = msg.tracker_id;
    }
    else {
      resolved.tracker_id = ''
    }

    if (msg.frames !== undefined) {
      resolved.frames = msg.frames;
    }
    else {
      resolved.frames = []
    }

    if (msg.confs !== undefined) {
      resolved.confs = msg.confs;
    }
    else {
      resolved.confs = []
    }

    if (msg.transforms !== undefined) {
      resolved.transforms = new Array(msg.transforms.length);
      for (let i = 0; i < resolved.transforms.length; ++i) {
        resolved.transforms[i] = geometry_msgs.msg.Transform.Resolve(msg.transforms[i]);
      }
    }
    else {
      resolved.transforms = []
    }

    if (msg.projective !== undefined) {
      resolved.projective = new Array(msg.projective.length);
      for (let i = 0; i < resolved.projective.length; ++i) {
        resolved.projective[i] = geometry_msgs.msg.Vector3.Resolve(msg.projective[i]);
      }
    }
    else {
      resolved.projective = []
    }

    if (msg.center_of_mass !== undefined) {
      resolved.center_of_mass = geometry_msgs.msg.Vector3.Resolve(msg.center_of_mass)
    }
    else {
      resolved.center_of_mass = new geometry_msgs.msg.Vector3()
    }

    return resolved;
    }
};

module.exports = TrackerUser;

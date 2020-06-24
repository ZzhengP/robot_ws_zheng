// Auto-generated. Do not edit!

// (in-package openni2_tracker.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let TrackerUser = require('./TrackerUser.js');

//-----------------------------------------------------------

class TrackerUserArray {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.users = null;
      this.numUsers = null;
    }
    else {
      if (initObj.hasOwnProperty('users')) {
        this.users = initObj.users
      }
      else {
        this.users = [];
      }
      if (initObj.hasOwnProperty('numUsers')) {
        this.numUsers = initObj.numUsers
      }
      else {
        this.numUsers = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TrackerUserArray
    // Serialize message field [users]
    // Serialize the length for message field [users]
    bufferOffset = _serializer.uint32(obj.users.length, buffer, bufferOffset);
    obj.users.forEach((val) => {
      bufferOffset = TrackerUser.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [numUsers]
    bufferOffset = _serializer.int8(obj.numUsers, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TrackerUserArray
    let len;
    let data = new TrackerUserArray(null);
    // Deserialize message field [users]
    // Deserialize array length for message field [users]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.users = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.users[i] = TrackerUser.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [numUsers]
    data.numUsers = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.users.forEach((val) => {
      length += TrackerUser.getMessageSize(val);
    });
    return length + 5;
  }

  static datatype() {
    // Returns string type for a message object
    return 'openni2_tracker/TrackerUserArray';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '825d95b7c6acaad9dc439d2148963f2e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # TrackerUserArray: This message contains a vector of openni2_tracker/TrackerUser messages.
    openni2_tracker/TrackerUser[] users
    int8 numUsers
    
    ================================================================================
    MSG: openni2_tracker/TrackerUser
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
    const resolved = new TrackerUserArray(null);
    if (msg.users !== undefined) {
      resolved.users = new Array(msg.users.length);
      for (let i = 0; i < resolved.users.length; ++i) {
        resolved.users[i] = TrackerUser.Resolve(msg.users[i]);
      }
    }
    else {
      resolved.users = []
    }

    if (msg.numUsers !== undefined) {
      resolved.numUsers = msg.numUsers;
    }
    else {
      resolved.numUsers = 0
    }

    return resolved;
    }
};

module.exports = TrackerUserArray;

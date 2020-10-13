// Auto-generated. Do not edit!

// (in-package kinect2_tracker.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let bounding_box = require('./bounding_box.js');
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class user_points {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.users = null;
      this.people_points = null;
      this.boxes = null;
    }
    else {
      if (initObj.hasOwnProperty('users')) {
        this.users = initObj.users
      }
      else {
        this.users = [];
      }
      if (initObj.hasOwnProperty('people_points')) {
        this.people_points = initObj.people_points
      }
      else {
        this.people_points = [];
      }
      if (initObj.hasOwnProperty('boxes')) {
        this.boxes = initObj.boxes
      }
      else {
        this.boxes = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type user_points
    // Serialize message field [users]
    bufferOffset = _arraySerializer.uint8(obj.users, buffer, bufferOffset, null);
    // Serialize message field [people_points]
    // Serialize the length for message field [people_points]
    bufferOffset = _serializer.uint32(obj.people_points.length, buffer, bufferOffset);
    obj.people_points.forEach((val) => {
      bufferOffset = geometry_msgs.msg.PointStamped.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [boxes]
    // Serialize the length for message field [boxes]
    bufferOffset = _serializer.uint32(obj.boxes.length, buffer, bufferOffset);
    obj.boxes.forEach((val) => {
      bufferOffset = bounding_box.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type user_points
    let len;
    let data = new user_points(null);
    // Deserialize message field [users]
    data.users = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    // Deserialize message field [people_points]
    // Deserialize array length for message field [people_points]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.people_points = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.people_points[i] = geometry_msgs.msg.PointStamped.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [boxes]
    // Deserialize array length for message field [boxes]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.boxes = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.boxes[i] = bounding_box.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.users.length;
    object.people_points.forEach((val) => {
      length += geometry_msgs.msg.PointStamped.getMessageSize(val);
    });
    object.boxes.forEach((val) => {
      length += bounding_box.getMessageSize(val);
    });
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kinect2_tracker/user_points';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6624cab0be89cfdee984fa8acaafad2a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Message storing the points of users.
    # Header header
    uint8[] users
    geometry_msgs/PointStamped[] people_points
    bounding_box[] boxes
    ================================================================================
    MSG: geometry_msgs/PointStamped
    # This represents a Point with reference coordinate frame and timestamp
    Header header
    Point point
    
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
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: kinect2_tracker/bounding_box
    geometry_msgs/PointStamped min
    geometry_msgs/PointStamped max
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new user_points(null);
    if (msg.users !== undefined) {
      resolved.users = msg.users;
    }
    else {
      resolved.users = []
    }

    if (msg.people_points !== undefined) {
      resolved.people_points = new Array(msg.people_points.length);
      for (let i = 0; i < resolved.people_points.length; ++i) {
        resolved.people_points[i] = geometry_msgs.msg.PointStamped.Resolve(msg.people_points[i]);
      }
    }
    else {
      resolved.people_points = []
    }

    if (msg.boxes !== undefined) {
      resolved.boxes = new Array(msg.boxes.length);
      for (let i = 0; i < resolved.boxes.length; ++i) {
        resolved.boxes[i] = bounding_box.Resolve(msg.boxes[i]);
      }
    }
    else {
      resolved.boxes = []
    }

    return resolved;
    }
};

module.exports = user_points;

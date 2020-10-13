// Auto-generated. Do not edit!

// (in-package kinect2_tracker.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class bounding_box {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.min = null;
      this.max = null;
    }
    else {
      if (initObj.hasOwnProperty('min')) {
        this.min = initObj.min
      }
      else {
        this.min = new geometry_msgs.msg.PointStamped();
      }
      if (initObj.hasOwnProperty('max')) {
        this.max = initObj.max
      }
      else {
        this.max = new geometry_msgs.msg.PointStamped();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type bounding_box
    // Serialize message field [min]
    bufferOffset = geometry_msgs.msg.PointStamped.serialize(obj.min, buffer, bufferOffset);
    // Serialize message field [max]
    bufferOffset = geometry_msgs.msg.PointStamped.serialize(obj.max, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type bounding_box
    let len;
    let data = new bounding_box(null);
    // Deserialize message field [min]
    data.min = geometry_msgs.msg.PointStamped.deserialize(buffer, bufferOffset);
    // Deserialize message field [max]
    data.max = geometry_msgs.msg.PointStamped.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += geometry_msgs.msg.PointStamped.getMessageSize(object.min);
    length += geometry_msgs.msg.PointStamped.getMessageSize(object.max);
    return length;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kinect2_tracker/bounding_box';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5a9f041aa663bad83df903fb4bed6c5c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/PointStamped min
    geometry_msgs/PointStamped max
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new bounding_box(null);
    if (msg.min !== undefined) {
      resolved.min = geometry_msgs.msg.PointStamped.Resolve(msg.min)
    }
    else {
      resolved.min = new geometry_msgs.msg.PointStamped()
    }

    if (msg.max !== undefined) {
      resolved.max = geometry_msgs.msg.PointStamped.Resolve(msg.max)
    }
    else {
      resolved.max = new geometry_msgs.msg.PointStamped()
    }

    return resolved;
    }
};

module.exports = bounding_box;

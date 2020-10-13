// Auto-generated. Do not edit!

// (in-package openpose_ros_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let OpenPoseHuman = require('./OpenPoseHuman.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class OpenPoseHumanList {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.image_header = null;
      this.num_humans = null;
      this.human_list = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('image_header')) {
        this.image_header = initObj.image_header
      }
      else {
        this.image_header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('num_humans')) {
        this.num_humans = initObj.num_humans
      }
      else {
        this.num_humans = 0;
      }
      if (initObj.hasOwnProperty('human_list')) {
        this.human_list = initObj.human_list
      }
      else {
        this.human_list = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type OpenPoseHumanList
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [image_header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.image_header, buffer, bufferOffset);
    // Serialize message field [num_humans]
    bufferOffset = _serializer.int32(obj.num_humans, buffer, bufferOffset);
    // Serialize message field [human_list]
    // Serialize the length for message field [human_list]
    bufferOffset = _serializer.uint32(obj.human_list.length, buffer, bufferOffset);
    obj.human_list.forEach((val) => {
      bufferOffset = OpenPoseHuman.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type OpenPoseHumanList
    let len;
    let data = new OpenPoseHumanList(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [image_header]
    data.image_header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [num_humans]
    data.num_humans = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [human_list]
    // Deserialize array length for message field [human_list]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.human_list = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.human_list[i] = OpenPoseHuman.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += std_msgs.msg.Header.getMessageSize(object.image_header);
    length += 176 * object.human_list.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'openpose_ros_msgs/OpenPoseHumanList';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '520d81a8dc226a535b021aef637b0340';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    Header image_header
    int32 num_humans
    OpenPoseHuman[] human_list
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
    MSG: openpose_ros_msgs/OpenPoseHuman
    int32 num_body_key_points_with_non_zero_prob
    int32 num_face_key_points_with_non_zero_prob
    int32 num_right_hand_key_points_with_non_zero_prob
    int32 num_left_hand_key_points_with_non_zero_prob
    BoundingBox body_bounding_box
    BoundingBox face_bounding_box
    PointWithProb[25] body_key_points_with_prob
    PointWithProb[70] face_key_points_with_prob
    PointWithProb[21] right_hand_key_points_with_prob
    PointWithProb[21] left_hand_key_points_with_prob
    ================================================================================
    MSG: openpose_ros_msgs/BoundingBox
    float64 x
    float64 y
    float64 width
    float64 height
    ================================================================================
    MSG: openpose_ros_msgs/PointWithProb
    float64 x
    float64 y
    float64 prob
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new OpenPoseHumanList(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.image_header !== undefined) {
      resolved.image_header = std_msgs.msg.Header.Resolve(msg.image_header)
    }
    else {
      resolved.image_header = new std_msgs.msg.Header()
    }

    if (msg.num_humans !== undefined) {
      resolved.num_humans = msg.num_humans;
    }
    else {
      resolved.num_humans = 0
    }

    if (msg.human_list !== undefined) {
      resolved.human_list = new Array(msg.human_list.length);
      for (let i = 0; i < resolved.human_list.length; ++i) {
        resolved.human_list[i] = OpenPoseHuman.Resolve(msg.human_list[i]);
      }
    }
    else {
      resolved.human_list = []
    }

    return resolved;
    }
};

module.exports = OpenPoseHumanList;

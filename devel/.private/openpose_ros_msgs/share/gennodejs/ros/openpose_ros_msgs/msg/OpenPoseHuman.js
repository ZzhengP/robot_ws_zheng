// Auto-generated. Do not edit!

// (in-package openpose_ros_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let BoundingBox = require('./BoundingBox.js');
let PointWithProb = require('./PointWithProb.js');

//-----------------------------------------------------------

class OpenPoseHuman {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.num_body_key_points_with_non_zero_prob = null;
      this.num_face_key_points_with_non_zero_prob = null;
      this.num_right_hand_key_points_with_non_zero_prob = null;
      this.num_left_hand_key_points_with_non_zero_prob = null;
      this.body_bounding_box = null;
      this.face_bounding_box = null;
      this.body_key_points_with_prob = null;
      this.face_key_points_with_prob = null;
      this.right_hand_key_points_with_prob = null;
      this.left_hand_key_points_with_prob = null;
    }
    else {
      if (initObj.hasOwnProperty('num_body_key_points_with_non_zero_prob')) {
        this.num_body_key_points_with_non_zero_prob = initObj.num_body_key_points_with_non_zero_prob
      }
      else {
        this.num_body_key_points_with_non_zero_prob = 0;
      }
      if (initObj.hasOwnProperty('num_face_key_points_with_non_zero_prob')) {
        this.num_face_key_points_with_non_zero_prob = initObj.num_face_key_points_with_non_zero_prob
      }
      else {
        this.num_face_key_points_with_non_zero_prob = 0;
      }
      if (initObj.hasOwnProperty('num_right_hand_key_points_with_non_zero_prob')) {
        this.num_right_hand_key_points_with_non_zero_prob = initObj.num_right_hand_key_points_with_non_zero_prob
      }
      else {
        this.num_right_hand_key_points_with_non_zero_prob = 0;
      }
      if (initObj.hasOwnProperty('num_left_hand_key_points_with_non_zero_prob')) {
        this.num_left_hand_key_points_with_non_zero_prob = initObj.num_left_hand_key_points_with_non_zero_prob
      }
      else {
        this.num_left_hand_key_points_with_non_zero_prob = 0;
      }
      if (initObj.hasOwnProperty('body_bounding_box')) {
        this.body_bounding_box = initObj.body_bounding_box
      }
      else {
        this.body_bounding_box = new BoundingBox();
      }
      if (initObj.hasOwnProperty('face_bounding_box')) {
        this.face_bounding_box = initObj.face_bounding_box
      }
      else {
        this.face_bounding_box = new BoundingBox();
      }
      if (initObj.hasOwnProperty('body_key_points_with_prob')) {
        this.body_key_points_with_prob = initObj.body_key_points_with_prob
      }
      else {
        this.body_key_points_with_prob = new Array(25).fill(new PointWithProb());
      }
      if (initObj.hasOwnProperty('face_key_points_with_prob')) {
        this.face_key_points_with_prob = initObj.face_key_points_with_prob
      }
      else {
        this.face_key_points_with_prob = new Array(70).fill(new PointWithProb());
      }
      if (initObj.hasOwnProperty('right_hand_key_points_with_prob')) {
        this.right_hand_key_points_with_prob = initObj.right_hand_key_points_with_prob
      }
      else {
        this.right_hand_key_points_with_prob = new Array(21).fill(new PointWithProb());
      }
      if (initObj.hasOwnProperty('left_hand_key_points_with_prob')) {
        this.left_hand_key_points_with_prob = initObj.left_hand_key_points_with_prob
      }
      else {
        this.left_hand_key_points_with_prob = new Array(21).fill(new PointWithProb());
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type OpenPoseHuman
    // Serialize message field [num_body_key_points_with_non_zero_prob]
    bufferOffset = _serializer.int32(obj.num_body_key_points_with_non_zero_prob, buffer, bufferOffset);
    // Serialize message field [num_face_key_points_with_non_zero_prob]
    bufferOffset = _serializer.int32(obj.num_face_key_points_with_non_zero_prob, buffer, bufferOffset);
    // Serialize message field [num_right_hand_key_points_with_non_zero_prob]
    bufferOffset = _serializer.int32(obj.num_right_hand_key_points_with_non_zero_prob, buffer, bufferOffset);
    // Serialize message field [num_left_hand_key_points_with_non_zero_prob]
    bufferOffset = _serializer.int32(obj.num_left_hand_key_points_with_non_zero_prob, buffer, bufferOffset);
    // Serialize message field [body_bounding_box]
    bufferOffset = BoundingBox.serialize(obj.body_bounding_box, buffer, bufferOffset);
    // Serialize message field [face_bounding_box]
    bufferOffset = BoundingBox.serialize(obj.face_bounding_box, buffer, bufferOffset);
    // Check that the constant length array field [body_key_points_with_prob] has the right length
    if (obj.body_key_points_with_prob.length !== 25) {
      throw new Error('Unable to serialize array field body_key_points_with_prob - length must be 25')
    }
    // Serialize message field [body_key_points_with_prob]
    obj.body_key_points_with_prob.forEach((val) => {
      bufferOffset = PointWithProb.serialize(val, buffer, bufferOffset);
    });
    // Check that the constant length array field [face_key_points_with_prob] has the right length
    if (obj.face_key_points_with_prob.length !== 70) {
      throw new Error('Unable to serialize array field face_key_points_with_prob - length must be 70')
    }
    // Serialize message field [face_key_points_with_prob]
    obj.face_key_points_with_prob.forEach((val) => {
      bufferOffset = PointWithProb.serialize(val, buffer, bufferOffset);
    });
    // Check that the constant length array field [right_hand_key_points_with_prob] has the right length
    if (obj.right_hand_key_points_with_prob.length !== 21) {
      throw new Error('Unable to serialize array field right_hand_key_points_with_prob - length must be 21')
    }
    // Serialize message field [right_hand_key_points_with_prob]
    obj.right_hand_key_points_with_prob.forEach((val) => {
      bufferOffset = PointWithProb.serialize(val, buffer, bufferOffset);
    });
    // Check that the constant length array field [left_hand_key_points_with_prob] has the right length
    if (obj.left_hand_key_points_with_prob.length !== 21) {
      throw new Error('Unable to serialize array field left_hand_key_points_with_prob - length must be 21')
    }
    // Serialize message field [left_hand_key_points_with_prob]
    obj.left_hand_key_points_with_prob.forEach((val) => {
      bufferOffset = PointWithProb.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type OpenPoseHuman
    let len;
    let data = new OpenPoseHuman(null);
    // Deserialize message field [num_body_key_points_with_non_zero_prob]
    data.num_body_key_points_with_non_zero_prob = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [num_face_key_points_with_non_zero_prob]
    data.num_face_key_points_with_non_zero_prob = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [num_right_hand_key_points_with_non_zero_prob]
    data.num_right_hand_key_points_with_non_zero_prob = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [num_left_hand_key_points_with_non_zero_prob]
    data.num_left_hand_key_points_with_non_zero_prob = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [body_bounding_box]
    data.body_bounding_box = BoundingBox.deserialize(buffer, bufferOffset);
    // Deserialize message field [face_bounding_box]
    data.face_bounding_box = BoundingBox.deserialize(buffer, bufferOffset);
    // Deserialize message field [body_key_points_with_prob]
    len = 25;
    data.body_key_points_with_prob = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.body_key_points_with_prob[i] = PointWithProb.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [face_key_points_with_prob]
    len = 70;
    data.face_key_points_with_prob = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.face_key_points_with_prob[i] = PointWithProb.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [right_hand_key_points_with_prob]
    len = 21;
    data.right_hand_key_points_with_prob = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.right_hand_key_points_with_prob[i] = PointWithProb.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [left_hand_key_points_with_prob]
    len = 21;
    data.left_hand_key_points_with_prob = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.left_hand_key_points_with_prob[i] = PointWithProb.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    return 176;
  }

  static datatype() {
    // Returns string type for a message object
    return 'openpose_ros_msgs/OpenPoseHuman';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4e291ca1063e4a96ca852f33fa35d22b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new OpenPoseHuman(null);
    if (msg.num_body_key_points_with_non_zero_prob !== undefined) {
      resolved.num_body_key_points_with_non_zero_prob = msg.num_body_key_points_with_non_zero_prob;
    }
    else {
      resolved.num_body_key_points_with_non_zero_prob = 0
    }

    if (msg.num_face_key_points_with_non_zero_prob !== undefined) {
      resolved.num_face_key_points_with_non_zero_prob = msg.num_face_key_points_with_non_zero_prob;
    }
    else {
      resolved.num_face_key_points_with_non_zero_prob = 0
    }

    if (msg.num_right_hand_key_points_with_non_zero_prob !== undefined) {
      resolved.num_right_hand_key_points_with_non_zero_prob = msg.num_right_hand_key_points_with_non_zero_prob;
    }
    else {
      resolved.num_right_hand_key_points_with_non_zero_prob = 0
    }

    if (msg.num_left_hand_key_points_with_non_zero_prob !== undefined) {
      resolved.num_left_hand_key_points_with_non_zero_prob = msg.num_left_hand_key_points_with_non_zero_prob;
    }
    else {
      resolved.num_left_hand_key_points_with_non_zero_prob = 0
    }

    if (msg.body_bounding_box !== undefined) {
      resolved.body_bounding_box = BoundingBox.Resolve(msg.body_bounding_box)
    }
    else {
      resolved.body_bounding_box = new BoundingBox()
    }

    if (msg.face_bounding_box !== undefined) {
      resolved.face_bounding_box = BoundingBox.Resolve(msg.face_bounding_box)
    }
    else {
      resolved.face_bounding_box = new BoundingBox()
    }

    if (msg.body_key_points_with_prob !== undefined) {
      resolved.body_key_points_with_prob = new Array(25)
      for (let i = 0; i < resolved.body_key_points_with_prob.length; ++i) {
        if (msg.body_key_points_with_prob.length > i) {
          resolved.body_key_points_with_prob[i] = PointWithProb.Resolve(msg.body_key_points_with_prob[i]);
        }
        else {
          resolved.body_key_points_with_prob[i] = new PointWithProb();
        }
      }
    }
    else {
      resolved.body_key_points_with_prob = new Array(25).fill(new PointWithProb())
    }

    if (msg.face_key_points_with_prob !== undefined) {
      resolved.face_key_points_with_prob = new Array(70)
      for (let i = 0; i < resolved.face_key_points_with_prob.length; ++i) {
        if (msg.face_key_points_with_prob.length > i) {
          resolved.face_key_points_with_prob[i] = PointWithProb.Resolve(msg.face_key_points_with_prob[i]);
        }
        else {
          resolved.face_key_points_with_prob[i] = new PointWithProb();
        }
      }
    }
    else {
      resolved.face_key_points_with_prob = new Array(70).fill(new PointWithProb())
    }

    if (msg.right_hand_key_points_with_prob !== undefined) {
      resolved.right_hand_key_points_with_prob = new Array(21)
      for (let i = 0; i < resolved.right_hand_key_points_with_prob.length; ++i) {
        if (msg.right_hand_key_points_with_prob.length > i) {
          resolved.right_hand_key_points_with_prob[i] = PointWithProb.Resolve(msg.right_hand_key_points_with_prob[i]);
        }
        else {
          resolved.right_hand_key_points_with_prob[i] = new PointWithProb();
        }
      }
    }
    else {
      resolved.right_hand_key_points_with_prob = new Array(21).fill(new PointWithProb())
    }

    if (msg.left_hand_key_points_with_prob !== undefined) {
      resolved.left_hand_key_points_with_prob = new Array(21)
      for (let i = 0; i < resolved.left_hand_key_points_with_prob.length; ++i) {
        if (msg.left_hand_key_points_with_prob.length > i) {
          resolved.left_hand_key_points_with_prob[i] = PointWithProb.Resolve(msg.left_hand_key_points_with_prob[i]);
        }
        else {
          resolved.left_hand_key_points_with_prob[i] = new PointWithProb();
        }
      }
    }
    else {
      resolved.left_hand_key_points_with_prob = new Array(21).fill(new PointWithProb())
    }

    return resolved;
    }
};

module.exports = OpenPoseHuman;

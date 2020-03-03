// Auto-generated. Do not edit!

// (in-package openpose_ros_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let BodyPartDetection = require('./BodyPartDetection.js');

//-----------------------------------------------------------

class PersonDetection {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.face_landmark = null;
      this.body_part = null;
    }
    else {
      if (initObj.hasOwnProperty('face_landmark')) {
        this.face_landmark = initObj.face_landmark
      }
      else {
        this.face_landmark = [];
      }
      if (initObj.hasOwnProperty('body_part')) {
        this.body_part = initObj.body_part
      }
      else {
        this.body_part = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PersonDetection
    // Serialize message field [face_landmark]
    // Serialize the length for message field [face_landmark]
    bufferOffset = _serializer.uint32(obj.face_landmark.length, buffer, bufferOffset);
    obj.face_landmark.forEach((val) => {
      bufferOffset = BodyPartDetection.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [body_part]
    // Serialize the length for message field [body_part]
    bufferOffset = _serializer.uint32(obj.body_part.length, buffer, bufferOffset);
    obj.body_part.forEach((val) => {
      bufferOffset = BodyPartDetection.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PersonDetection
    let len;
    let data = new PersonDetection(null);
    // Deserialize message field [face_landmark]
    // Deserialize array length for message field [face_landmark]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.face_landmark = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.face_landmark[i] = BodyPartDetection.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [body_part]
    // Deserialize array length for message field [body_part]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.body_part = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.body_part[i] = BodyPartDetection.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 16 * object.face_landmark.length;
    length += 16 * object.body_part.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'openpose_ros_msgs/PersonDetection';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5cf57469f872bf144c62ed7904d5c159';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    BodyPartDetection[] face_landmark
    BodyPartDetection[] body_part
    
    ================================================================================
    MSG: openpose_ros_msgs/BodyPartDetection
    uint32 part_id
    uint32 x
    uint32 y
    float32 confidence
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PersonDetection(null);
    if (msg.face_landmark !== undefined) {
      resolved.face_landmark = new Array(msg.face_landmark.length);
      for (let i = 0; i < resolved.face_landmark.length; ++i) {
        resolved.face_landmark[i] = BodyPartDetection.Resolve(msg.face_landmark[i]);
      }
    }
    else {
      resolved.face_landmark = []
    }

    if (msg.body_part !== undefined) {
      resolved.body_part = new Array(msg.body_part.length);
      for (let i = 0; i < resolved.body_part.length; ++i) {
        resolved.body_part[i] = BodyPartDetection.Resolve(msg.body_part[i]);
      }
    }
    else {
      resolved.body_part = []
    }

    return resolved;
    }
};

module.exports = PersonDetection;

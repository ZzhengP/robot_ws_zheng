// Auto-generated. Do not edit!

// (in-package openpose_ros_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let PersonDetection = require('./PersonDetection.js');

//-----------------------------------------------------------

class Persons {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.persons = null;
      this.rostime = null;
      this.image_w = null;
      this.image_h = null;
    }
    else {
      if (initObj.hasOwnProperty('persons')) {
        this.persons = initObj.persons
      }
      else {
        this.persons = [];
      }
      if (initObj.hasOwnProperty('rostime')) {
        this.rostime = initObj.rostime
      }
      else {
        this.rostime = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('image_w')) {
        this.image_w = initObj.image_w
      }
      else {
        this.image_w = 0;
      }
      if (initObj.hasOwnProperty('image_h')) {
        this.image_h = initObj.image_h
      }
      else {
        this.image_h = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Persons
    // Serialize message field [persons]
    // Serialize the length for message field [persons]
    bufferOffset = _serializer.uint32(obj.persons.length, buffer, bufferOffset);
    obj.persons.forEach((val) => {
      bufferOffset = PersonDetection.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [rostime]
    bufferOffset = _serializer.time(obj.rostime, buffer, bufferOffset);
    // Serialize message field [image_w]
    bufferOffset = _serializer.uint32(obj.image_w, buffer, bufferOffset);
    // Serialize message field [image_h]
    bufferOffset = _serializer.uint32(obj.image_h, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Persons
    let len;
    let data = new Persons(null);
    // Deserialize message field [persons]
    // Deserialize array length for message field [persons]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.persons = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.persons[i] = PersonDetection.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [rostime]
    data.rostime = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [image_w]
    data.image_w = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [image_h]
    data.image_h = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.persons.forEach((val) => {
      length += PersonDetection.getMessageSize(val);
    });
    return length + 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'openpose_ros_msgs/Persons';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '364e552085331e28feea9ffef91f2929';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    PersonDetection[] persons
    time rostime
    uint32 image_w
    uint32 image_h
    
    ================================================================================
    MSG: openpose_ros_msgs/PersonDetection
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
    const resolved = new Persons(null);
    if (msg.persons !== undefined) {
      resolved.persons = new Array(msg.persons.length);
      for (let i = 0; i < resolved.persons.length; ++i) {
        resolved.persons[i] = PersonDetection.Resolve(msg.persons[i]);
      }
    }
    else {
      resolved.persons = []
    }

    if (msg.rostime !== undefined) {
      resolved.rostime = msg.rostime;
    }
    else {
      resolved.rostime = {secs: 0, nsecs: 0}
    }

    if (msg.image_w !== undefined) {
      resolved.image_w = msg.image_w;
    }
    else {
      resolved.image_w = 0
    }

    if (msg.image_h !== undefined) {
      resolved.image_h = msg.image_h;
    }
    else {
      resolved.image_h = 0
    }

    return resolved;
    }
};

module.exports = Persons;

// Auto-generated. Do not edit!

// (in-package kinect2_tracker.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class user_IDs {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.users = null;
    }
    else {
      if (initObj.hasOwnProperty('users')) {
        this.users = initObj.users
      }
      else {
        this.users = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type user_IDs
    // Serialize message field [users]
    bufferOffset = _arraySerializer.uint8(obj.users, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type user_IDs
    let len;
    let data = new user_IDs(null);
    // Deserialize message field [users]
    data.users = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.users.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kinect2_tracker/user_IDs';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0370fea64fb82134b7a78a66cb25797c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8[] users
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new user_IDs(null);
    if (msg.users !== undefined) {
      resolved.users = msg.users;
    }
    else {
      resolved.users = []
    }

    return resolved;
    }
};

module.exports = user_IDs;

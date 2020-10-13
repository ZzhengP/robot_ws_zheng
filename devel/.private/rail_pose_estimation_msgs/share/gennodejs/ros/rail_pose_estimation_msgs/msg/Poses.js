// Auto-generated. Do not edit!

// (in-package rail_pose_estimation_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Keypoints = require('./Keypoints.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Poses {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.people = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('people')) {
        this.people = initObj.people
      }
      else {
        this.people = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Poses
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [people]
    // Serialize the length for message field [people]
    bufferOffset = _serializer.uint32(obj.people.length, buffer, bufferOffset);
    obj.people.forEach((val) => {
      bufferOffset = Keypoints.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Poses
    let len;
    let data = new Poses(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [people]
    // Deserialize array length for message field [people]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.people = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.people[i] = Keypoints.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 144 * object.people.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rail_pose_estimation_msgs/Poses';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f3ac5a5326099357214a8296eb96e5d2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header 	# Message header including timestamp of detection & img frame
    Keypoints[] people 			# Array of detected objects
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
    MSG: rail_pose_estimation_msgs/Keypoints
    float32 neck_x                  # x coord of neck
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
    float32 left_ear_y              # y coord of left ear
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Poses(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.people !== undefined) {
      resolved.people = new Array(msg.people.length);
      for (let i = 0; i < resolved.people.length; ++i) {
        resolved.people[i] = Keypoints.Resolve(msg.people[i]);
      }
    }
    else {
      resolved.people = []
    }

    return resolved;
    }
};

module.exports = Poses;

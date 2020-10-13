// Auto-generated. Do not edit!

// (in-package rail_pose_estimation_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Keypoints {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.neck_x = null;
      this.neck_y = null;
      this.nose_x = null;
      this.nose_y = null;
      this.right_shoulder_x = null;
      this.right_shoulder_y = null;
      this.left_shoulder_x = null;
      this.left_shoulder_y = null;
      this.right_elbow_x = null;
      this.right_elbow_y = null;
      this.left_elbow_x = null;
      this.left_elbow_y = null;
      this.right_wrist_x = null;
      this.right_wrist_y = null;
      this.left_wrist_x = null;
      this.left_wrist_y = null;
      this.right_hip_x = null;
      this.right_hip_y = null;
      this.left_hip_x = null;
      this.left_hip_y = null;
      this.right_knee_x = null;
      this.right_knee_y = null;
      this.left_knee_x = null;
      this.left_knee_y = null;
      this.right_ankle_x = null;
      this.right_ankle_y = null;
      this.left_ankle_x = null;
      this.left_ankle_y = null;
      this.right_eye_x = null;
      this.right_eye_y = null;
      this.left_eye_x = null;
      this.left_eye_y = null;
      this.right_ear_x = null;
      this.right_ear_y = null;
      this.left_ear_x = null;
      this.left_ear_y = null;
    }
    else {
      if (initObj.hasOwnProperty('neck_x')) {
        this.neck_x = initObj.neck_x
      }
      else {
        this.neck_x = 0.0;
      }
      if (initObj.hasOwnProperty('neck_y')) {
        this.neck_y = initObj.neck_y
      }
      else {
        this.neck_y = 0.0;
      }
      if (initObj.hasOwnProperty('nose_x')) {
        this.nose_x = initObj.nose_x
      }
      else {
        this.nose_x = 0.0;
      }
      if (initObj.hasOwnProperty('nose_y')) {
        this.nose_y = initObj.nose_y
      }
      else {
        this.nose_y = 0.0;
      }
      if (initObj.hasOwnProperty('right_shoulder_x')) {
        this.right_shoulder_x = initObj.right_shoulder_x
      }
      else {
        this.right_shoulder_x = 0.0;
      }
      if (initObj.hasOwnProperty('right_shoulder_y')) {
        this.right_shoulder_y = initObj.right_shoulder_y
      }
      else {
        this.right_shoulder_y = 0.0;
      }
      if (initObj.hasOwnProperty('left_shoulder_x')) {
        this.left_shoulder_x = initObj.left_shoulder_x
      }
      else {
        this.left_shoulder_x = 0.0;
      }
      if (initObj.hasOwnProperty('left_shoulder_y')) {
        this.left_shoulder_y = initObj.left_shoulder_y
      }
      else {
        this.left_shoulder_y = 0.0;
      }
      if (initObj.hasOwnProperty('right_elbow_x')) {
        this.right_elbow_x = initObj.right_elbow_x
      }
      else {
        this.right_elbow_x = 0.0;
      }
      if (initObj.hasOwnProperty('right_elbow_y')) {
        this.right_elbow_y = initObj.right_elbow_y
      }
      else {
        this.right_elbow_y = 0.0;
      }
      if (initObj.hasOwnProperty('left_elbow_x')) {
        this.left_elbow_x = initObj.left_elbow_x
      }
      else {
        this.left_elbow_x = 0.0;
      }
      if (initObj.hasOwnProperty('left_elbow_y')) {
        this.left_elbow_y = initObj.left_elbow_y
      }
      else {
        this.left_elbow_y = 0.0;
      }
      if (initObj.hasOwnProperty('right_wrist_x')) {
        this.right_wrist_x = initObj.right_wrist_x
      }
      else {
        this.right_wrist_x = 0.0;
      }
      if (initObj.hasOwnProperty('right_wrist_y')) {
        this.right_wrist_y = initObj.right_wrist_y
      }
      else {
        this.right_wrist_y = 0.0;
      }
      if (initObj.hasOwnProperty('left_wrist_x')) {
        this.left_wrist_x = initObj.left_wrist_x
      }
      else {
        this.left_wrist_x = 0.0;
      }
      if (initObj.hasOwnProperty('left_wrist_y')) {
        this.left_wrist_y = initObj.left_wrist_y
      }
      else {
        this.left_wrist_y = 0.0;
      }
      if (initObj.hasOwnProperty('right_hip_x')) {
        this.right_hip_x = initObj.right_hip_x
      }
      else {
        this.right_hip_x = 0.0;
      }
      if (initObj.hasOwnProperty('right_hip_y')) {
        this.right_hip_y = initObj.right_hip_y
      }
      else {
        this.right_hip_y = 0.0;
      }
      if (initObj.hasOwnProperty('left_hip_x')) {
        this.left_hip_x = initObj.left_hip_x
      }
      else {
        this.left_hip_x = 0.0;
      }
      if (initObj.hasOwnProperty('left_hip_y')) {
        this.left_hip_y = initObj.left_hip_y
      }
      else {
        this.left_hip_y = 0.0;
      }
      if (initObj.hasOwnProperty('right_knee_x')) {
        this.right_knee_x = initObj.right_knee_x
      }
      else {
        this.right_knee_x = 0.0;
      }
      if (initObj.hasOwnProperty('right_knee_y')) {
        this.right_knee_y = initObj.right_knee_y
      }
      else {
        this.right_knee_y = 0.0;
      }
      if (initObj.hasOwnProperty('left_knee_x')) {
        this.left_knee_x = initObj.left_knee_x
      }
      else {
        this.left_knee_x = 0.0;
      }
      if (initObj.hasOwnProperty('left_knee_y')) {
        this.left_knee_y = initObj.left_knee_y
      }
      else {
        this.left_knee_y = 0.0;
      }
      if (initObj.hasOwnProperty('right_ankle_x')) {
        this.right_ankle_x = initObj.right_ankle_x
      }
      else {
        this.right_ankle_x = 0.0;
      }
      if (initObj.hasOwnProperty('right_ankle_y')) {
        this.right_ankle_y = initObj.right_ankle_y
      }
      else {
        this.right_ankle_y = 0.0;
      }
      if (initObj.hasOwnProperty('left_ankle_x')) {
        this.left_ankle_x = initObj.left_ankle_x
      }
      else {
        this.left_ankle_x = 0.0;
      }
      if (initObj.hasOwnProperty('left_ankle_y')) {
        this.left_ankle_y = initObj.left_ankle_y
      }
      else {
        this.left_ankle_y = 0.0;
      }
      if (initObj.hasOwnProperty('right_eye_x')) {
        this.right_eye_x = initObj.right_eye_x
      }
      else {
        this.right_eye_x = 0.0;
      }
      if (initObj.hasOwnProperty('right_eye_y')) {
        this.right_eye_y = initObj.right_eye_y
      }
      else {
        this.right_eye_y = 0.0;
      }
      if (initObj.hasOwnProperty('left_eye_x')) {
        this.left_eye_x = initObj.left_eye_x
      }
      else {
        this.left_eye_x = 0.0;
      }
      if (initObj.hasOwnProperty('left_eye_y')) {
        this.left_eye_y = initObj.left_eye_y
      }
      else {
        this.left_eye_y = 0.0;
      }
      if (initObj.hasOwnProperty('right_ear_x')) {
        this.right_ear_x = initObj.right_ear_x
      }
      else {
        this.right_ear_x = 0.0;
      }
      if (initObj.hasOwnProperty('right_ear_y')) {
        this.right_ear_y = initObj.right_ear_y
      }
      else {
        this.right_ear_y = 0.0;
      }
      if (initObj.hasOwnProperty('left_ear_x')) {
        this.left_ear_x = initObj.left_ear_x
      }
      else {
        this.left_ear_x = 0.0;
      }
      if (initObj.hasOwnProperty('left_ear_y')) {
        this.left_ear_y = initObj.left_ear_y
      }
      else {
        this.left_ear_y = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Keypoints
    // Serialize message field [neck_x]
    bufferOffset = _serializer.float32(obj.neck_x, buffer, bufferOffset);
    // Serialize message field [neck_y]
    bufferOffset = _serializer.float32(obj.neck_y, buffer, bufferOffset);
    // Serialize message field [nose_x]
    bufferOffset = _serializer.float32(obj.nose_x, buffer, bufferOffset);
    // Serialize message field [nose_y]
    bufferOffset = _serializer.float32(obj.nose_y, buffer, bufferOffset);
    // Serialize message field [right_shoulder_x]
    bufferOffset = _serializer.float32(obj.right_shoulder_x, buffer, bufferOffset);
    // Serialize message field [right_shoulder_y]
    bufferOffset = _serializer.float32(obj.right_shoulder_y, buffer, bufferOffset);
    // Serialize message field [left_shoulder_x]
    bufferOffset = _serializer.float32(obj.left_shoulder_x, buffer, bufferOffset);
    // Serialize message field [left_shoulder_y]
    bufferOffset = _serializer.float32(obj.left_shoulder_y, buffer, bufferOffset);
    // Serialize message field [right_elbow_x]
    bufferOffset = _serializer.float32(obj.right_elbow_x, buffer, bufferOffset);
    // Serialize message field [right_elbow_y]
    bufferOffset = _serializer.float32(obj.right_elbow_y, buffer, bufferOffset);
    // Serialize message field [left_elbow_x]
    bufferOffset = _serializer.float32(obj.left_elbow_x, buffer, bufferOffset);
    // Serialize message field [left_elbow_y]
    bufferOffset = _serializer.float32(obj.left_elbow_y, buffer, bufferOffset);
    // Serialize message field [right_wrist_x]
    bufferOffset = _serializer.float32(obj.right_wrist_x, buffer, bufferOffset);
    // Serialize message field [right_wrist_y]
    bufferOffset = _serializer.float32(obj.right_wrist_y, buffer, bufferOffset);
    // Serialize message field [left_wrist_x]
    bufferOffset = _serializer.float32(obj.left_wrist_x, buffer, bufferOffset);
    // Serialize message field [left_wrist_y]
    bufferOffset = _serializer.float32(obj.left_wrist_y, buffer, bufferOffset);
    // Serialize message field [right_hip_x]
    bufferOffset = _serializer.float32(obj.right_hip_x, buffer, bufferOffset);
    // Serialize message field [right_hip_y]
    bufferOffset = _serializer.float32(obj.right_hip_y, buffer, bufferOffset);
    // Serialize message field [left_hip_x]
    bufferOffset = _serializer.float32(obj.left_hip_x, buffer, bufferOffset);
    // Serialize message field [left_hip_y]
    bufferOffset = _serializer.float32(obj.left_hip_y, buffer, bufferOffset);
    // Serialize message field [right_knee_x]
    bufferOffset = _serializer.float32(obj.right_knee_x, buffer, bufferOffset);
    // Serialize message field [right_knee_y]
    bufferOffset = _serializer.float32(obj.right_knee_y, buffer, bufferOffset);
    // Serialize message field [left_knee_x]
    bufferOffset = _serializer.float32(obj.left_knee_x, buffer, bufferOffset);
    // Serialize message field [left_knee_y]
    bufferOffset = _serializer.float32(obj.left_knee_y, buffer, bufferOffset);
    // Serialize message field [right_ankle_x]
    bufferOffset = _serializer.float32(obj.right_ankle_x, buffer, bufferOffset);
    // Serialize message field [right_ankle_y]
    bufferOffset = _serializer.float32(obj.right_ankle_y, buffer, bufferOffset);
    // Serialize message field [left_ankle_x]
    bufferOffset = _serializer.float32(obj.left_ankle_x, buffer, bufferOffset);
    // Serialize message field [left_ankle_y]
    bufferOffset = _serializer.float32(obj.left_ankle_y, buffer, bufferOffset);
    // Serialize message field [right_eye_x]
    bufferOffset = _serializer.float32(obj.right_eye_x, buffer, bufferOffset);
    // Serialize message field [right_eye_y]
    bufferOffset = _serializer.float32(obj.right_eye_y, buffer, bufferOffset);
    // Serialize message field [left_eye_x]
    bufferOffset = _serializer.float32(obj.left_eye_x, buffer, bufferOffset);
    // Serialize message field [left_eye_y]
    bufferOffset = _serializer.float32(obj.left_eye_y, buffer, bufferOffset);
    // Serialize message field [right_ear_x]
    bufferOffset = _serializer.float32(obj.right_ear_x, buffer, bufferOffset);
    // Serialize message field [right_ear_y]
    bufferOffset = _serializer.float32(obj.right_ear_y, buffer, bufferOffset);
    // Serialize message field [left_ear_x]
    bufferOffset = _serializer.float32(obj.left_ear_x, buffer, bufferOffset);
    // Serialize message field [left_ear_y]
    bufferOffset = _serializer.float32(obj.left_ear_y, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Keypoints
    let len;
    let data = new Keypoints(null);
    // Deserialize message field [neck_x]
    data.neck_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [neck_y]
    data.neck_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [nose_x]
    data.nose_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [nose_y]
    data.nose_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [right_shoulder_x]
    data.right_shoulder_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [right_shoulder_y]
    data.right_shoulder_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [left_shoulder_x]
    data.left_shoulder_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [left_shoulder_y]
    data.left_shoulder_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [right_elbow_x]
    data.right_elbow_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [right_elbow_y]
    data.right_elbow_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [left_elbow_x]
    data.left_elbow_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [left_elbow_y]
    data.left_elbow_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [right_wrist_x]
    data.right_wrist_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [right_wrist_y]
    data.right_wrist_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [left_wrist_x]
    data.left_wrist_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [left_wrist_y]
    data.left_wrist_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [right_hip_x]
    data.right_hip_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [right_hip_y]
    data.right_hip_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [left_hip_x]
    data.left_hip_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [left_hip_y]
    data.left_hip_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [right_knee_x]
    data.right_knee_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [right_knee_y]
    data.right_knee_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [left_knee_x]
    data.left_knee_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [left_knee_y]
    data.left_knee_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [right_ankle_x]
    data.right_ankle_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [right_ankle_y]
    data.right_ankle_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [left_ankle_x]
    data.left_ankle_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [left_ankle_y]
    data.left_ankle_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [right_eye_x]
    data.right_eye_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [right_eye_y]
    data.right_eye_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [left_eye_x]
    data.left_eye_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [left_eye_y]
    data.left_eye_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [right_ear_x]
    data.right_ear_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [right_ear_y]
    data.right_ear_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [left_ear_x]
    data.left_ear_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [left_ear_y]
    data.left_ear_y = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 144;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rail_pose_estimation_msgs/Keypoints';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3d1804a99352b413ee0c9ca364640114';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new Keypoints(null);
    if (msg.neck_x !== undefined) {
      resolved.neck_x = msg.neck_x;
    }
    else {
      resolved.neck_x = 0.0
    }

    if (msg.neck_y !== undefined) {
      resolved.neck_y = msg.neck_y;
    }
    else {
      resolved.neck_y = 0.0
    }

    if (msg.nose_x !== undefined) {
      resolved.nose_x = msg.nose_x;
    }
    else {
      resolved.nose_x = 0.0
    }

    if (msg.nose_y !== undefined) {
      resolved.nose_y = msg.nose_y;
    }
    else {
      resolved.nose_y = 0.0
    }

    if (msg.right_shoulder_x !== undefined) {
      resolved.right_shoulder_x = msg.right_shoulder_x;
    }
    else {
      resolved.right_shoulder_x = 0.0
    }

    if (msg.right_shoulder_y !== undefined) {
      resolved.right_shoulder_y = msg.right_shoulder_y;
    }
    else {
      resolved.right_shoulder_y = 0.0
    }

    if (msg.left_shoulder_x !== undefined) {
      resolved.left_shoulder_x = msg.left_shoulder_x;
    }
    else {
      resolved.left_shoulder_x = 0.0
    }

    if (msg.left_shoulder_y !== undefined) {
      resolved.left_shoulder_y = msg.left_shoulder_y;
    }
    else {
      resolved.left_shoulder_y = 0.0
    }

    if (msg.right_elbow_x !== undefined) {
      resolved.right_elbow_x = msg.right_elbow_x;
    }
    else {
      resolved.right_elbow_x = 0.0
    }

    if (msg.right_elbow_y !== undefined) {
      resolved.right_elbow_y = msg.right_elbow_y;
    }
    else {
      resolved.right_elbow_y = 0.0
    }

    if (msg.left_elbow_x !== undefined) {
      resolved.left_elbow_x = msg.left_elbow_x;
    }
    else {
      resolved.left_elbow_x = 0.0
    }

    if (msg.left_elbow_y !== undefined) {
      resolved.left_elbow_y = msg.left_elbow_y;
    }
    else {
      resolved.left_elbow_y = 0.0
    }

    if (msg.right_wrist_x !== undefined) {
      resolved.right_wrist_x = msg.right_wrist_x;
    }
    else {
      resolved.right_wrist_x = 0.0
    }

    if (msg.right_wrist_y !== undefined) {
      resolved.right_wrist_y = msg.right_wrist_y;
    }
    else {
      resolved.right_wrist_y = 0.0
    }

    if (msg.left_wrist_x !== undefined) {
      resolved.left_wrist_x = msg.left_wrist_x;
    }
    else {
      resolved.left_wrist_x = 0.0
    }

    if (msg.left_wrist_y !== undefined) {
      resolved.left_wrist_y = msg.left_wrist_y;
    }
    else {
      resolved.left_wrist_y = 0.0
    }

    if (msg.right_hip_x !== undefined) {
      resolved.right_hip_x = msg.right_hip_x;
    }
    else {
      resolved.right_hip_x = 0.0
    }

    if (msg.right_hip_y !== undefined) {
      resolved.right_hip_y = msg.right_hip_y;
    }
    else {
      resolved.right_hip_y = 0.0
    }

    if (msg.left_hip_x !== undefined) {
      resolved.left_hip_x = msg.left_hip_x;
    }
    else {
      resolved.left_hip_x = 0.0
    }

    if (msg.left_hip_y !== undefined) {
      resolved.left_hip_y = msg.left_hip_y;
    }
    else {
      resolved.left_hip_y = 0.0
    }

    if (msg.right_knee_x !== undefined) {
      resolved.right_knee_x = msg.right_knee_x;
    }
    else {
      resolved.right_knee_x = 0.0
    }

    if (msg.right_knee_y !== undefined) {
      resolved.right_knee_y = msg.right_knee_y;
    }
    else {
      resolved.right_knee_y = 0.0
    }

    if (msg.left_knee_x !== undefined) {
      resolved.left_knee_x = msg.left_knee_x;
    }
    else {
      resolved.left_knee_x = 0.0
    }

    if (msg.left_knee_y !== undefined) {
      resolved.left_knee_y = msg.left_knee_y;
    }
    else {
      resolved.left_knee_y = 0.0
    }

    if (msg.right_ankle_x !== undefined) {
      resolved.right_ankle_x = msg.right_ankle_x;
    }
    else {
      resolved.right_ankle_x = 0.0
    }

    if (msg.right_ankle_y !== undefined) {
      resolved.right_ankle_y = msg.right_ankle_y;
    }
    else {
      resolved.right_ankle_y = 0.0
    }

    if (msg.left_ankle_x !== undefined) {
      resolved.left_ankle_x = msg.left_ankle_x;
    }
    else {
      resolved.left_ankle_x = 0.0
    }

    if (msg.left_ankle_y !== undefined) {
      resolved.left_ankle_y = msg.left_ankle_y;
    }
    else {
      resolved.left_ankle_y = 0.0
    }

    if (msg.right_eye_x !== undefined) {
      resolved.right_eye_x = msg.right_eye_x;
    }
    else {
      resolved.right_eye_x = 0.0
    }

    if (msg.right_eye_y !== undefined) {
      resolved.right_eye_y = msg.right_eye_y;
    }
    else {
      resolved.right_eye_y = 0.0
    }

    if (msg.left_eye_x !== undefined) {
      resolved.left_eye_x = msg.left_eye_x;
    }
    else {
      resolved.left_eye_x = 0.0
    }

    if (msg.left_eye_y !== undefined) {
      resolved.left_eye_y = msg.left_eye_y;
    }
    else {
      resolved.left_eye_y = 0.0
    }

    if (msg.right_ear_x !== undefined) {
      resolved.right_ear_x = msg.right_ear_x;
    }
    else {
      resolved.right_ear_x = 0.0
    }

    if (msg.right_ear_y !== undefined) {
      resolved.right_ear_y = msg.right_ear_y;
    }
    else {
      resolved.right_ear_y = 0.0
    }

    if (msg.left_ear_x !== undefined) {
      resolved.left_ear_x = msg.left_ear_x;
    }
    else {
      resolved.left_ear_x = 0.0
    }

    if (msg.left_ear_y !== undefined) {
      resolved.left_ear_y = msg.left_ear_y;
    }
    else {
      resolved.left_ear_y = 0.0
    }

    return resolved;
    }
};

module.exports = Keypoints;

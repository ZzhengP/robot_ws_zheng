// Auto-generated. Do not edit!

// (in-package human_moveit_config.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let sensor_msgs = _finder('sensor_msgs');
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class GetHumanIKRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.desired_poses = null;
      this.fixed_joints = null;
      this.tolerance = null;
      this.group_names = null;
      this.seed = null;
    }
    else {
      if (initObj.hasOwnProperty('desired_poses')) {
        this.desired_poses = initObj.desired_poses
      }
      else {
        this.desired_poses = [];
      }
      if (initObj.hasOwnProperty('fixed_joints')) {
        this.fixed_joints = initObj.fixed_joints
      }
      else {
        this.fixed_joints = new sensor_msgs.msg.JointState();
      }
      if (initObj.hasOwnProperty('tolerance')) {
        this.tolerance = initObj.tolerance
      }
      else {
        this.tolerance = 0.0;
      }
      if (initObj.hasOwnProperty('group_names')) {
        this.group_names = initObj.group_names
      }
      else {
        this.group_names = [];
      }
      if (initObj.hasOwnProperty('seed')) {
        this.seed = initObj.seed
      }
      else {
        this.seed = new sensor_msgs.msg.JointState();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetHumanIKRequest
    // Serialize message field [desired_poses]
    // Serialize the length for message field [desired_poses]
    bufferOffset = _serializer.uint32(obj.desired_poses.length, buffer, bufferOffset);
    obj.desired_poses.forEach((val) => {
      bufferOffset = geometry_msgs.msg.PoseStamped.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [fixed_joints]
    bufferOffset = sensor_msgs.msg.JointState.serialize(obj.fixed_joints, buffer, bufferOffset);
    // Serialize message field [tolerance]
    bufferOffset = _serializer.float32(obj.tolerance, buffer, bufferOffset);
    // Serialize message field [group_names]
    bufferOffset = _arraySerializer.string(obj.group_names, buffer, bufferOffset, null);
    // Serialize message field [seed]
    bufferOffset = sensor_msgs.msg.JointState.serialize(obj.seed, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetHumanIKRequest
    let len;
    let data = new GetHumanIKRequest(null);
    // Deserialize message field [desired_poses]
    // Deserialize array length for message field [desired_poses]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.desired_poses = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.desired_poses[i] = geometry_msgs.msg.PoseStamped.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [fixed_joints]
    data.fixed_joints = sensor_msgs.msg.JointState.deserialize(buffer, bufferOffset);
    // Deserialize message field [tolerance]
    data.tolerance = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [group_names]
    data.group_names = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [seed]
    data.seed = sensor_msgs.msg.JointState.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.desired_poses.forEach((val) => {
      length += geometry_msgs.msg.PoseStamped.getMessageSize(val);
    });
    length += sensor_msgs.msg.JointState.getMessageSize(object.fixed_joints);
    object.group_names.forEach((val) => {
      length += 4 + val.length;
    });
    length += sensor_msgs.msg.JointState.getMessageSize(object.seed);
    return length + 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'human_moveit_config/GetHumanIKRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '570f111db5ed3535ac213f7c5fd26f8c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    
    
    
    
    
    
    
    
    geometry_msgs/PoseStamped[] desired_poses
    sensor_msgs/JointState fixed_joints
    float32 tolerance
    string[] group_names
    sensor_msgs/JointState seed
    
    ================================================================================
    MSG: geometry_msgs/PoseStamped
    # A Pose with reference coordinate frame and timestamp
    Header header
    Pose pose
    
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
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
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
    
    ================================================================================
    MSG: sensor_msgs/JointState
    # This is a message that holds data to describe the state of a set of torque controlled joints. 
    #
    # The state of each joint (revolute or prismatic) is defined by:
    #  * the position of the joint (rad or m),
    #  * the velocity of the joint (rad/s or m/s) and 
    #  * the effort that is applied in the joint (Nm or N).
    #
    # Each joint is uniquely identified by its name
    # The header specifies the time at which the joint states were recorded. All the joint states
    # in one message have to be recorded at the same time.
    #
    # This message consists of a multiple arrays, one for each part of the joint state. 
    # The goal is to make each of the fields optional. When e.g. your joints have no
    # effort associated with them, you can leave the effort array empty. 
    #
    # All arrays in this message should have the same size, or be empty.
    # This is the only way to uniquely associate the joint name with the correct
    # states.
    
    
    Header header
    
    string[] name
    float64[] position
    float64[] velocity
    float64[] effort
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetHumanIKRequest(null);
    if (msg.desired_poses !== undefined) {
      resolved.desired_poses = new Array(msg.desired_poses.length);
      for (let i = 0; i < resolved.desired_poses.length; ++i) {
        resolved.desired_poses[i] = geometry_msgs.msg.PoseStamped.Resolve(msg.desired_poses[i]);
      }
    }
    else {
      resolved.desired_poses = []
    }

    if (msg.fixed_joints !== undefined) {
      resolved.fixed_joints = sensor_msgs.msg.JointState.Resolve(msg.fixed_joints)
    }
    else {
      resolved.fixed_joints = new sensor_msgs.msg.JointState()
    }

    if (msg.tolerance !== undefined) {
      resolved.tolerance = msg.tolerance;
    }
    else {
      resolved.tolerance = 0.0
    }

    if (msg.group_names !== undefined) {
      resolved.group_names = msg.group_names;
    }
    else {
      resolved.group_names = []
    }

    if (msg.seed !== undefined) {
      resolved.seed = sensor_msgs.msg.JointState.Resolve(msg.seed)
    }
    else {
      resolved.seed = new sensor_msgs.msg.JointState()
    }

    return resolved;
    }
};

class GetHumanIKResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.joint_state = null;
    }
    else {
      if (initObj.hasOwnProperty('joint_state')) {
        this.joint_state = initObj.joint_state
      }
      else {
        this.joint_state = new sensor_msgs.msg.JointState();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetHumanIKResponse
    // Serialize message field [joint_state]
    bufferOffset = sensor_msgs.msg.JointState.serialize(obj.joint_state, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetHumanIKResponse
    let len;
    let data = new GetHumanIKResponse(null);
    // Deserialize message field [joint_state]
    data.joint_state = sensor_msgs.msg.JointState.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += sensor_msgs.msg.JointState.getMessageSize(object.joint_state);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'human_moveit_config/GetHumanIKResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9ca061465ef0ed08771ed240c43789f5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    sensor_msgs/JointState joint_state
    
    ================================================================================
    MSG: sensor_msgs/JointState
    # This is a message that holds data to describe the state of a set of torque controlled joints. 
    #
    # The state of each joint (revolute or prismatic) is defined by:
    #  * the position of the joint (rad or m),
    #  * the velocity of the joint (rad/s or m/s) and 
    #  * the effort that is applied in the joint (Nm or N).
    #
    # Each joint is uniquely identified by its name
    # The header specifies the time at which the joint states were recorded. All the joint states
    # in one message have to be recorded at the same time.
    #
    # This message consists of a multiple arrays, one for each part of the joint state. 
    # The goal is to make each of the fields optional. When e.g. your joints have no
    # effort associated with them, you can leave the effort array empty. 
    #
    # All arrays in this message should have the same size, or be empty.
    # This is the only way to uniquely associate the joint name with the correct
    # states.
    
    
    Header header
    
    string[] name
    float64[] position
    float64[] velocity
    float64[] effort
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetHumanIKResponse(null);
    if (msg.joint_state !== undefined) {
      resolved.joint_state = sensor_msgs.msg.JointState.Resolve(msg.joint_state)
    }
    else {
      resolved.joint_state = new sensor_msgs.msg.JointState()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetHumanIKRequest,
  Response: GetHumanIKResponse,
  md5sum() { return '2602509d5e7f9ded6d3747732576ab30'; },
  datatype() { return 'human_moveit_config/GetHumanIK'; }
};

// Auto-generated. Do not edit!

// (in-package robotic_deposition.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class ur5e_data {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.joint_angles = null;
      this.joint_velocities = null;
      this.tcp_pose = null;
      this.tcp_speed = null;
      this.tcp_force = null;
      this.time = null;
      this.is_steady = null;
    }
    else {
      if (initObj.hasOwnProperty('joint_angles')) {
        this.joint_angles = initObj.joint_angles
      }
      else {
        this.joint_angles = new std_msgs.msg.Float64MultiArray();
      }
      if (initObj.hasOwnProperty('joint_velocities')) {
        this.joint_velocities = initObj.joint_velocities
      }
      else {
        this.joint_velocities = new std_msgs.msg.Float64MultiArray();
      }
      if (initObj.hasOwnProperty('tcp_pose')) {
        this.tcp_pose = initObj.tcp_pose
      }
      else {
        this.tcp_pose = new std_msgs.msg.Float64MultiArray();
      }
      if (initObj.hasOwnProperty('tcp_speed')) {
        this.tcp_speed = initObj.tcp_speed
      }
      else {
        this.tcp_speed = new std_msgs.msg.Float64MultiArray();
      }
      if (initObj.hasOwnProperty('tcp_force')) {
        this.tcp_force = initObj.tcp_force
      }
      else {
        this.tcp_force = new std_msgs.msg.Float64MultiArray();
      }
      if (initObj.hasOwnProperty('time')) {
        this.time = initObj.time
      }
      else {
        this.time = new std_msgs.msg.Float64();
      }
      if (initObj.hasOwnProperty('is_steady')) {
        this.is_steady = initObj.is_steady
      }
      else {
        this.is_steady = new std_msgs.msg.Bool();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ur5e_data
    // Serialize message field [joint_angles]
    bufferOffset = std_msgs.msg.Float64MultiArray.serialize(obj.joint_angles, buffer, bufferOffset);
    // Serialize message field [joint_velocities]
    bufferOffset = std_msgs.msg.Float64MultiArray.serialize(obj.joint_velocities, buffer, bufferOffset);
    // Serialize message field [tcp_pose]
    bufferOffset = std_msgs.msg.Float64MultiArray.serialize(obj.tcp_pose, buffer, bufferOffset);
    // Serialize message field [tcp_speed]
    bufferOffset = std_msgs.msg.Float64MultiArray.serialize(obj.tcp_speed, buffer, bufferOffset);
    // Serialize message field [tcp_force]
    bufferOffset = std_msgs.msg.Float64MultiArray.serialize(obj.tcp_force, buffer, bufferOffset);
    // Serialize message field [time]
    bufferOffset = std_msgs.msg.Float64.serialize(obj.time, buffer, bufferOffset);
    // Serialize message field [is_steady]
    bufferOffset = std_msgs.msg.Bool.serialize(obj.is_steady, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ur5e_data
    let len;
    let data = new ur5e_data(null);
    // Deserialize message field [joint_angles]
    data.joint_angles = std_msgs.msg.Float64MultiArray.deserialize(buffer, bufferOffset);
    // Deserialize message field [joint_velocities]
    data.joint_velocities = std_msgs.msg.Float64MultiArray.deserialize(buffer, bufferOffset);
    // Deserialize message field [tcp_pose]
    data.tcp_pose = std_msgs.msg.Float64MultiArray.deserialize(buffer, bufferOffset);
    // Deserialize message field [tcp_speed]
    data.tcp_speed = std_msgs.msg.Float64MultiArray.deserialize(buffer, bufferOffset);
    // Deserialize message field [tcp_force]
    data.tcp_force = std_msgs.msg.Float64MultiArray.deserialize(buffer, bufferOffset);
    // Deserialize message field [time]
    data.time = std_msgs.msg.Float64.deserialize(buffer, bufferOffset);
    // Deserialize message field [is_steady]
    data.is_steady = std_msgs.msg.Bool.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Float64MultiArray.getMessageSize(object.joint_angles);
    length += std_msgs.msg.Float64MultiArray.getMessageSize(object.joint_velocities);
    length += std_msgs.msg.Float64MultiArray.getMessageSize(object.tcp_pose);
    length += std_msgs.msg.Float64MultiArray.getMessageSize(object.tcp_speed);
    length += std_msgs.msg.Float64MultiArray.getMessageSize(object.tcp_force);
    return length + 9;
  }

  static datatype() {
    // Returns string type for a message object
    return 'robotic_deposition/ur5e_data';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '273f5ae00ca10a4d616012f38fd5c175';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Float64MultiArray joint_angles
    std_msgs/Float64MultiArray joint_velocities
    std_msgs/Float64MultiArray tcp_pose
    std_msgs/Float64MultiArray tcp_speed
    std_msgs/Float64MultiArray tcp_force
    std_msgs/Float64  time
    std_msgs/Bool is_steady
    ================================================================================
    MSG: std_msgs/Float64MultiArray
    # Please look at the MultiArrayLayout message definition for
    # documentation on all multiarrays.
    
    MultiArrayLayout  layout        # specification of data layout
    float64[]         data          # array of data
    
    
    ================================================================================
    MSG: std_msgs/MultiArrayLayout
    # The multiarray declares a generic multi-dimensional array of a
    # particular data type.  Dimensions are ordered from outer most
    # to inner most.
    
    MultiArrayDimension[] dim # Array of dimension properties
    uint32 data_offset        # padding elements at front of data
    
    # Accessors should ALWAYS be written in terms of dimension stride
    # and specified outer-most dimension first.
    # 
    # multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
    #
    # A standard, 3-channel 640x480 image with interleaved color channels
    # would be specified as:
    #
    # dim[0].label  = "height"
    # dim[0].size   = 480
    # dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
    # dim[1].label  = "width"
    # dim[1].size   = 640
    # dim[1].stride = 3*640 = 1920
    # dim[2].label  = "channel"
    # dim[2].size   = 3
    # dim[2].stride = 3
    #
    # multiarray(i,j,k) refers to the ith row, jth column, and kth channel.
    
    ================================================================================
    MSG: std_msgs/MultiArrayDimension
    string label   # label of given dimension
    uint32 size    # size of given dimension (in type units)
    uint32 stride  # stride of given dimension
    ================================================================================
    MSG: std_msgs/Float64
    float64 data
    ================================================================================
    MSG: std_msgs/Bool
    bool data
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ur5e_data(null);
    if (msg.joint_angles !== undefined) {
      resolved.joint_angles = std_msgs.msg.Float64MultiArray.Resolve(msg.joint_angles)
    }
    else {
      resolved.joint_angles = new std_msgs.msg.Float64MultiArray()
    }

    if (msg.joint_velocities !== undefined) {
      resolved.joint_velocities = std_msgs.msg.Float64MultiArray.Resolve(msg.joint_velocities)
    }
    else {
      resolved.joint_velocities = new std_msgs.msg.Float64MultiArray()
    }

    if (msg.tcp_pose !== undefined) {
      resolved.tcp_pose = std_msgs.msg.Float64MultiArray.Resolve(msg.tcp_pose)
    }
    else {
      resolved.tcp_pose = new std_msgs.msg.Float64MultiArray()
    }

    if (msg.tcp_speed !== undefined) {
      resolved.tcp_speed = std_msgs.msg.Float64MultiArray.Resolve(msg.tcp_speed)
    }
    else {
      resolved.tcp_speed = new std_msgs.msg.Float64MultiArray()
    }

    if (msg.tcp_force !== undefined) {
      resolved.tcp_force = std_msgs.msg.Float64MultiArray.Resolve(msg.tcp_force)
    }
    else {
      resolved.tcp_force = new std_msgs.msg.Float64MultiArray()
    }

    if (msg.time !== undefined) {
      resolved.time = std_msgs.msg.Float64.Resolve(msg.time)
    }
    else {
      resolved.time = new std_msgs.msg.Float64()
    }

    if (msg.is_steady !== undefined) {
      resolved.is_steady = std_msgs.msg.Bool.Resolve(msg.is_steady)
    }
    else {
      resolved.is_steady = new std_msgs.msg.Bool()
    }

    return resolved;
    }
};

module.exports = ur5e_data;

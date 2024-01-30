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

class ur5e_control {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.inputs = null;
      this.command_type = null;
    }
    else {
      if (initObj.hasOwnProperty('inputs')) {
        this.inputs = initObj.inputs
      }
      else {
        this.inputs = new std_msgs.msg.Float64MultiArray();
      }
      if (initObj.hasOwnProperty('command_type')) {
        this.command_type = initObj.command_type
      }
      else {
        this.command_type = new std_msgs.msg.String();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ur5e_control
    // Serialize message field [inputs]
    bufferOffset = std_msgs.msg.Float64MultiArray.serialize(obj.inputs, buffer, bufferOffset);
    // Serialize message field [command_type]
    bufferOffset = std_msgs.msg.String.serialize(obj.command_type, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ur5e_control
    let len;
    let data = new ur5e_control(null);
    // Deserialize message field [inputs]
    data.inputs = std_msgs.msg.Float64MultiArray.deserialize(buffer, bufferOffset);
    // Deserialize message field [command_type]
    data.command_type = std_msgs.msg.String.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Float64MultiArray.getMessageSize(object.inputs);
    length += std_msgs.msg.String.getMessageSize(object.command_type);
    return length;
  }

  static datatype() {
    // Returns string type for a message object
    return 'robotic_deposition/ur5e_control';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6af43574263174a907cc0088df6dfae1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Float64MultiArray inputs
    std_msgs/String  command_type
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
    MSG: std_msgs/String
    string data
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ur5e_control(null);
    if (msg.inputs !== undefined) {
      resolved.inputs = std_msgs.msg.Float64MultiArray.Resolve(msg.inputs)
    }
    else {
      resolved.inputs = new std_msgs.msg.Float64MultiArray()
    }

    if (msg.command_type !== undefined) {
      resolved.command_type = std_msgs.msg.String.Resolve(msg.command_type)
    }
    else {
      resolved.command_type = new std_msgs.msg.String()
    }

    return resolved;
    }
};

module.exports = ur5e_control;

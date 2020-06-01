// Auto-generated. Do not edit!

// (in-package nubot_pummba_msg.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class PummbaCmd {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.vel_linear = null;
      this.vel_angular = null;
      this.front_left = null;
      this.front_right = null;
      this.rear_left = null;
      this.rear_right = null;
    }
    else {
      if (initObj.hasOwnProperty('vel_linear')) {
        this.vel_linear = initObj.vel_linear
      }
      else {
        this.vel_linear = 0.0;
      }
      if (initObj.hasOwnProperty('vel_angular')) {
        this.vel_angular = initObj.vel_angular
      }
      else {
        this.vel_angular = 0.0;
      }
      if (initObj.hasOwnProperty('front_left')) {
        this.front_left = initObj.front_left
      }
      else {
        this.front_left = 0.0;
      }
      if (initObj.hasOwnProperty('front_right')) {
        this.front_right = initObj.front_right
      }
      else {
        this.front_right = 0.0;
      }
      if (initObj.hasOwnProperty('rear_left')) {
        this.rear_left = initObj.rear_left
      }
      else {
        this.rear_left = 0.0;
      }
      if (initObj.hasOwnProperty('rear_right')) {
        this.rear_right = initObj.rear_right
      }
      else {
        this.rear_right = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PummbaCmd
    // Serialize message field [vel_linear]
    bufferOffset = _serializer.float32(obj.vel_linear, buffer, bufferOffset);
    // Serialize message field [vel_angular]
    bufferOffset = _serializer.float32(obj.vel_angular, buffer, bufferOffset);
    // Serialize message field [front_left]
    bufferOffset = _serializer.float32(obj.front_left, buffer, bufferOffset);
    // Serialize message field [front_right]
    bufferOffset = _serializer.float32(obj.front_right, buffer, bufferOffset);
    // Serialize message field [rear_left]
    bufferOffset = _serializer.float32(obj.rear_left, buffer, bufferOffset);
    // Serialize message field [rear_right]
    bufferOffset = _serializer.float32(obj.rear_right, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PummbaCmd
    let len;
    let data = new PummbaCmd(null);
    // Deserialize message field [vel_linear]
    data.vel_linear = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [vel_angular]
    data.vel_angular = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [front_left]
    data.front_left = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [front_right]
    data.front_right = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [rear_left]
    data.rear_left = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [rear_right]
    data.rear_right = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'nubot_pummba_msg/PummbaCmd';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '37f5dcb42b8ba6407dc6ea2389a5de4c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 vel_linear
    float32 vel_angular
    float32 front_left
    float32 front_right
    float32 rear_left
    float32 rear_right
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PummbaCmd(null);
    if (msg.vel_linear !== undefined) {
      resolved.vel_linear = msg.vel_linear;
    }
    else {
      resolved.vel_linear = 0.0
    }

    if (msg.vel_angular !== undefined) {
      resolved.vel_angular = msg.vel_angular;
    }
    else {
      resolved.vel_angular = 0.0
    }

    if (msg.front_left !== undefined) {
      resolved.front_left = msg.front_left;
    }
    else {
      resolved.front_left = 0.0
    }

    if (msg.front_right !== undefined) {
      resolved.front_right = msg.front_right;
    }
    else {
      resolved.front_right = 0.0
    }

    if (msg.rear_left !== undefined) {
      resolved.rear_left = msg.rear_left;
    }
    else {
      resolved.rear_left = 0.0
    }

    if (msg.rear_right !== undefined) {
      resolved.rear_right = msg.rear_right;
    }
    else {
      resolved.rear_right = 0.0
    }

    return resolved;
    }
};

module.exports = PummbaCmd;

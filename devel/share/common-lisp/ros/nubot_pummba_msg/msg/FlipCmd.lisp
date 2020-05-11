; Auto-generated. Do not edit!


(cl:in-package nubot_pummba_msg-msg)


;//! \htmlinclude FlipCmd.msg.html

(cl:defclass <FlipCmd> (roslisp-msg-protocol:ros-message)
  ((front_left
    :reader front_left
    :initarg :front_left
    :type cl:float
    :initform 0.0)
   (front_right
    :reader front_right
    :initarg :front_right
    :type cl:float
    :initform 0.0)
   (rear_left
    :reader rear_left
    :initarg :rear_left
    :type cl:float
    :initform 0.0)
   (rear_right
    :reader rear_right
    :initarg :rear_right
    :type cl:float
    :initform 0.0))
)

(cl:defclass FlipCmd (<FlipCmd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FlipCmd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FlipCmd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name nubot_pummba_msg-msg:<FlipCmd> is deprecated: use nubot_pummba_msg-msg:FlipCmd instead.")))

(cl:ensure-generic-function 'front_left-val :lambda-list '(m))
(cl:defmethod front_left-val ((m <FlipCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nubot_pummba_msg-msg:front_left-val is deprecated.  Use nubot_pummba_msg-msg:front_left instead.")
  (front_left m))

(cl:ensure-generic-function 'front_right-val :lambda-list '(m))
(cl:defmethod front_right-val ((m <FlipCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nubot_pummba_msg-msg:front_right-val is deprecated.  Use nubot_pummba_msg-msg:front_right instead.")
  (front_right m))

(cl:ensure-generic-function 'rear_left-val :lambda-list '(m))
(cl:defmethod rear_left-val ((m <FlipCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nubot_pummba_msg-msg:rear_left-val is deprecated.  Use nubot_pummba_msg-msg:rear_left instead.")
  (rear_left m))

(cl:ensure-generic-function 'rear_right-val :lambda-list '(m))
(cl:defmethod rear_right-val ((m <FlipCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nubot_pummba_msg-msg:rear_right-val is deprecated.  Use nubot_pummba_msg-msg:rear_right instead.")
  (rear_right m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FlipCmd>) ostream)
  "Serializes a message object of type '<FlipCmd>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'front_left))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'front_right))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rear_left))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rear_right))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FlipCmd>) istream)
  "Deserializes a message object of type '<FlipCmd>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'front_left) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'front_right) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rear_left) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rear_right) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FlipCmd>)))
  "Returns string type for a message object of type '<FlipCmd>"
  "nubot_pummba_msg/FlipCmd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FlipCmd)))
  "Returns string type for a message object of type 'FlipCmd"
  "nubot_pummba_msg/FlipCmd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FlipCmd>)))
  "Returns md5sum for a message object of type '<FlipCmd>"
  "704d6e45a144b051d9261eee9f265122")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FlipCmd)))
  "Returns md5sum for a message object of type 'FlipCmd"
  "704d6e45a144b051d9261eee9f265122")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FlipCmd>)))
  "Returns full string definition for message of type '<FlipCmd>"
  (cl:format cl:nil "float32 front_left~%float32 front_right~%float32 rear_left~%float32 rear_right~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FlipCmd)))
  "Returns full string definition for message of type 'FlipCmd"
  (cl:format cl:nil "float32 front_left~%float32 front_right~%float32 rear_left~%float32 rear_right~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FlipCmd>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FlipCmd>))
  "Converts a ROS message object to a list"
  (cl:list 'FlipCmd
    (cl:cons ':front_left (front_left msg))
    (cl:cons ':front_right (front_right msg))
    (cl:cons ':rear_left (rear_left msg))
    (cl:cons ':rear_right (rear_right msg))
))

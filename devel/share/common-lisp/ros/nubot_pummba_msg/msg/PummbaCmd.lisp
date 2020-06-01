; Auto-generated. Do not edit!


(cl:in-package nubot_pummba_msg-msg)


;//! \htmlinclude PummbaCmd.msg.html

(cl:defclass <PummbaCmd> (roslisp-msg-protocol:ros-message)
  ((vel_linear
    :reader vel_linear
    :initarg :vel_linear
    :type cl:float
    :initform 0.0)
   (vel_angular
    :reader vel_angular
    :initarg :vel_angular
    :type cl:float
    :initform 0.0)
   (front_left
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

(cl:defclass PummbaCmd (<PummbaCmd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PummbaCmd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PummbaCmd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name nubot_pummba_msg-msg:<PummbaCmd> is deprecated: use nubot_pummba_msg-msg:PummbaCmd instead.")))

(cl:ensure-generic-function 'vel_linear-val :lambda-list '(m))
(cl:defmethod vel_linear-val ((m <PummbaCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nubot_pummba_msg-msg:vel_linear-val is deprecated.  Use nubot_pummba_msg-msg:vel_linear instead.")
  (vel_linear m))

(cl:ensure-generic-function 'vel_angular-val :lambda-list '(m))
(cl:defmethod vel_angular-val ((m <PummbaCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nubot_pummba_msg-msg:vel_angular-val is deprecated.  Use nubot_pummba_msg-msg:vel_angular instead.")
  (vel_angular m))

(cl:ensure-generic-function 'front_left-val :lambda-list '(m))
(cl:defmethod front_left-val ((m <PummbaCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nubot_pummba_msg-msg:front_left-val is deprecated.  Use nubot_pummba_msg-msg:front_left instead.")
  (front_left m))

(cl:ensure-generic-function 'front_right-val :lambda-list '(m))
(cl:defmethod front_right-val ((m <PummbaCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nubot_pummba_msg-msg:front_right-val is deprecated.  Use nubot_pummba_msg-msg:front_right instead.")
  (front_right m))

(cl:ensure-generic-function 'rear_left-val :lambda-list '(m))
(cl:defmethod rear_left-val ((m <PummbaCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nubot_pummba_msg-msg:rear_left-val is deprecated.  Use nubot_pummba_msg-msg:rear_left instead.")
  (rear_left m))

(cl:ensure-generic-function 'rear_right-val :lambda-list '(m))
(cl:defmethod rear_right-val ((m <PummbaCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nubot_pummba_msg-msg:rear_right-val is deprecated.  Use nubot_pummba_msg-msg:rear_right instead.")
  (rear_right m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PummbaCmd>) ostream)
  "Serializes a message object of type '<PummbaCmd>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vel_linear))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vel_angular))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PummbaCmd>) istream)
  "Deserializes a message object of type '<PummbaCmd>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vel_linear) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vel_angular) (roslisp-utils:decode-single-float-bits bits)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PummbaCmd>)))
  "Returns string type for a message object of type '<PummbaCmd>"
  "nubot_pummba_msg/PummbaCmd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PummbaCmd)))
  "Returns string type for a message object of type 'PummbaCmd"
  "nubot_pummba_msg/PummbaCmd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PummbaCmd>)))
  "Returns md5sum for a message object of type '<PummbaCmd>"
  "37f5dcb42b8ba6407dc6ea2389a5de4c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PummbaCmd)))
  "Returns md5sum for a message object of type 'PummbaCmd"
  "37f5dcb42b8ba6407dc6ea2389a5de4c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PummbaCmd>)))
  "Returns full string definition for message of type '<PummbaCmd>"
  (cl:format cl:nil "float32 vel_linear~%float32 vel_angular~%float32 front_left~%float32 front_right~%float32 rear_left~%float32 rear_right~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PummbaCmd)))
  "Returns full string definition for message of type 'PummbaCmd"
  (cl:format cl:nil "float32 vel_linear~%float32 vel_angular~%float32 front_left~%float32 front_right~%float32 rear_left~%float32 rear_right~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PummbaCmd>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PummbaCmd>))
  "Converts a ROS message object to a list"
  (cl:list 'PummbaCmd
    (cl:cons ':vel_linear (vel_linear msg))
    (cl:cons ':vel_angular (vel_angular msg))
    (cl:cons ':front_left (front_left msg))
    (cl:cons ':front_right (front_right msg))
    (cl:cons ':rear_left (rear_left msg))
    (cl:cons ':rear_right (rear_right msg))
))

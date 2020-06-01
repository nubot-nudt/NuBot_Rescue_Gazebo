;; Auto-generated. Do not edit!


(when (boundp 'nubot_pummba_msg::PummbaCmd)
  (if (not (find-package "NUBOT_PUMMBA_MSG"))
    (make-package "NUBOT_PUMMBA_MSG"))
  (shadow 'PummbaCmd (find-package "NUBOT_PUMMBA_MSG")))
(unless (find-package "NUBOT_PUMMBA_MSG::PUMMBACMD")
  (make-package "NUBOT_PUMMBA_MSG::PUMMBACMD"))

(in-package "ROS")
;;//! \htmlinclude PummbaCmd.msg.html


(defclass nubot_pummba_msg::PummbaCmd
  :super ros::object
  :slots (_vel_linear _vel_angular _front_left _front_right _rear_left _rear_right ))

(defmethod nubot_pummba_msg::PummbaCmd
  (:init
   (&key
    ((:vel_linear __vel_linear) 0.0)
    ((:vel_angular __vel_angular) 0.0)
    ((:front_left __front_left) 0.0)
    ((:front_right __front_right) 0.0)
    ((:rear_left __rear_left) 0.0)
    ((:rear_right __rear_right) 0.0)
    )
   (send-super :init)
   (setq _vel_linear (float __vel_linear))
   (setq _vel_angular (float __vel_angular))
   (setq _front_left (float __front_left))
   (setq _front_right (float __front_right))
   (setq _rear_left (float __rear_left))
   (setq _rear_right (float __rear_right))
   self)
  (:vel_linear
   (&optional __vel_linear)
   (if __vel_linear (setq _vel_linear __vel_linear)) _vel_linear)
  (:vel_angular
   (&optional __vel_angular)
   (if __vel_angular (setq _vel_angular __vel_angular)) _vel_angular)
  (:front_left
   (&optional __front_left)
   (if __front_left (setq _front_left __front_left)) _front_left)
  (:front_right
   (&optional __front_right)
   (if __front_right (setq _front_right __front_right)) _front_right)
  (:rear_left
   (&optional __rear_left)
   (if __rear_left (setq _rear_left __rear_left)) _rear_left)
  (:rear_right
   (&optional __rear_right)
   (if __rear_right (setq _rear_right __rear_right)) _rear_right)
  (:serialization-length
   ()
   (+
    ;; float32 _vel_linear
    4
    ;; float32 _vel_angular
    4
    ;; float32 _front_left
    4
    ;; float32 _front_right
    4
    ;; float32 _rear_left
    4
    ;; float32 _rear_right
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _vel_linear
       (sys::poke _vel_linear (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vel_angular
       (sys::poke _vel_angular (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _front_left
       (sys::poke _front_left (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _front_right
       (sys::poke _front_right (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _rear_left
       (sys::poke _rear_left (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _rear_right
       (sys::poke _rear_right (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _vel_linear
     (setq _vel_linear (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vel_angular
     (setq _vel_angular (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _front_left
     (setq _front_left (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _front_right
     (setq _front_right (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _rear_left
     (setq _rear_left (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _rear_right
     (setq _rear_right (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get nubot_pummba_msg::PummbaCmd :md5sum-) "37f5dcb42b8ba6407dc6ea2389a5de4c")
(setf (get nubot_pummba_msg::PummbaCmd :datatype-) "nubot_pummba_msg/PummbaCmd")
(setf (get nubot_pummba_msg::PummbaCmd :definition-)
      "float32 vel_linear
float32 vel_angular
float32 front_left
float32 front_right
float32 rear_left
float32 rear_right

")



(provide :nubot_pummba_msg/PummbaCmd "37f5dcb42b8ba6407dc6ea2389a5de4c")



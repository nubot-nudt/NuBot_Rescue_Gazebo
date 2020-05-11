;; Auto-generated. Do not edit!


(when (boundp 'nubot_pummba_msg::FlipCmd)
  (if (not (find-package "NUBOT_PUMMBA_MSG"))
    (make-package "NUBOT_PUMMBA_MSG"))
  (shadow 'FlipCmd (find-package "NUBOT_PUMMBA_MSG")))
(unless (find-package "NUBOT_PUMMBA_MSG::FLIPCMD")
  (make-package "NUBOT_PUMMBA_MSG::FLIPCMD"))

(in-package "ROS")
;;//! \htmlinclude FlipCmd.msg.html


(defclass nubot_pummba_msg::FlipCmd
  :super ros::object
  :slots (_front_left _front_right _rear_left _rear_right ))

(defmethod nubot_pummba_msg::FlipCmd
  (:init
   (&key
    ((:front_left __front_left) 0.0)
    ((:front_right __front_right) 0.0)
    ((:rear_left __rear_left) 0.0)
    ((:rear_right __rear_right) 0.0)
    )
   (send-super :init)
   (setq _front_left (float __front_left))
   (setq _front_right (float __front_right))
   (setq _rear_left (float __rear_left))
   (setq _rear_right (float __rear_right))
   self)
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

(setf (get nubot_pummba_msg::FlipCmd :md5sum-) "704d6e45a144b051d9261eee9f265122")
(setf (get nubot_pummba_msg::FlipCmd :datatype-) "nubot_pummba_msg/FlipCmd")
(setf (get nubot_pummba_msg::FlipCmd :definition-)
      "float32 front_left
float32 front_right
float32 rear_left
float32 rear_right

")



(provide :nubot_pummba_msg/FlipCmd "704d6e45a144b051d9261eee9f265122")



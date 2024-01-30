;; Auto-generated. Do not edit!


(when (boundp 'robotic_deposition::ur5e_data)
  (if (not (find-package "ROBOTIC_DEPOSITION"))
    (make-package "ROBOTIC_DEPOSITION"))
  (shadow 'ur5e_data (find-package "ROBOTIC_DEPOSITION")))
(unless (find-package "ROBOTIC_DEPOSITION::UR5E_DATA")
  (make-package "ROBOTIC_DEPOSITION::UR5E_DATA"))

(in-package "ROS")
;;//! \htmlinclude ur5e_data.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass robotic_deposition::ur5e_data
  :super ros::object
  :slots (_joint_angles _joint_velocities _tcp_pose _tcp_speed _tcp_force _time _is_steady ))

(defmethod robotic_deposition::ur5e_data
  (:init
   (&key
    ((:joint_angles __joint_angles) (instance std_msgs::Float64MultiArray :init))
    ((:joint_velocities __joint_velocities) (instance std_msgs::Float64MultiArray :init))
    ((:tcp_pose __tcp_pose) (instance std_msgs::Float64MultiArray :init))
    ((:tcp_speed __tcp_speed) (instance std_msgs::Float64MultiArray :init))
    ((:tcp_force __tcp_force) (instance std_msgs::Float64MultiArray :init))
    ((:time __time) (instance std_msgs::Float64 :init))
    ((:is_steady __is_steady) (instance std_msgs::Bool :init))
    )
   (send-super :init)
   (setq _joint_angles __joint_angles)
   (setq _joint_velocities __joint_velocities)
   (setq _tcp_pose __tcp_pose)
   (setq _tcp_speed __tcp_speed)
   (setq _tcp_force __tcp_force)
   (setq _time __time)
   (setq _is_steady __is_steady)
   self)
  (:joint_angles
   (&rest __joint_angles)
   (if (keywordp (car __joint_angles))
       (send* _joint_angles __joint_angles)
     (progn
       (if __joint_angles (setq _joint_angles (car __joint_angles)))
       _joint_angles)))
  (:joint_velocities
   (&rest __joint_velocities)
   (if (keywordp (car __joint_velocities))
       (send* _joint_velocities __joint_velocities)
     (progn
       (if __joint_velocities (setq _joint_velocities (car __joint_velocities)))
       _joint_velocities)))
  (:tcp_pose
   (&rest __tcp_pose)
   (if (keywordp (car __tcp_pose))
       (send* _tcp_pose __tcp_pose)
     (progn
       (if __tcp_pose (setq _tcp_pose (car __tcp_pose)))
       _tcp_pose)))
  (:tcp_speed
   (&rest __tcp_speed)
   (if (keywordp (car __tcp_speed))
       (send* _tcp_speed __tcp_speed)
     (progn
       (if __tcp_speed (setq _tcp_speed (car __tcp_speed)))
       _tcp_speed)))
  (:tcp_force
   (&rest __tcp_force)
   (if (keywordp (car __tcp_force))
       (send* _tcp_force __tcp_force)
     (progn
       (if __tcp_force (setq _tcp_force (car __tcp_force)))
       _tcp_force)))
  (:time
   (&rest __time)
   (if (keywordp (car __time))
       (send* _time __time)
     (progn
       (if __time (setq _time (car __time)))
       _time)))
  (:is_steady
   (&rest __is_steady)
   (if (keywordp (car __is_steady))
       (send* _is_steady __is_steady)
     (progn
       (if __is_steady (setq _is_steady (car __is_steady)))
       _is_steady)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Float64MultiArray _joint_angles
    (send _joint_angles :serialization-length)
    ;; std_msgs/Float64MultiArray _joint_velocities
    (send _joint_velocities :serialization-length)
    ;; std_msgs/Float64MultiArray _tcp_pose
    (send _tcp_pose :serialization-length)
    ;; std_msgs/Float64MultiArray _tcp_speed
    (send _tcp_speed :serialization-length)
    ;; std_msgs/Float64MultiArray _tcp_force
    (send _tcp_force :serialization-length)
    ;; std_msgs/Float64 _time
    (send _time :serialization-length)
    ;; std_msgs/Bool _is_steady
    (send _is_steady :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Float64MultiArray _joint_angles
       (send _joint_angles :serialize s)
     ;; std_msgs/Float64MultiArray _joint_velocities
       (send _joint_velocities :serialize s)
     ;; std_msgs/Float64MultiArray _tcp_pose
       (send _tcp_pose :serialize s)
     ;; std_msgs/Float64MultiArray _tcp_speed
       (send _tcp_speed :serialize s)
     ;; std_msgs/Float64MultiArray _tcp_force
       (send _tcp_force :serialize s)
     ;; std_msgs/Float64 _time
       (send _time :serialize s)
     ;; std_msgs/Bool _is_steady
       (send _is_steady :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Float64MultiArray _joint_angles
     (send _joint_angles :deserialize buf ptr-) (incf ptr- (send _joint_angles :serialization-length))
   ;; std_msgs/Float64MultiArray _joint_velocities
     (send _joint_velocities :deserialize buf ptr-) (incf ptr- (send _joint_velocities :serialization-length))
   ;; std_msgs/Float64MultiArray _tcp_pose
     (send _tcp_pose :deserialize buf ptr-) (incf ptr- (send _tcp_pose :serialization-length))
   ;; std_msgs/Float64MultiArray _tcp_speed
     (send _tcp_speed :deserialize buf ptr-) (incf ptr- (send _tcp_speed :serialization-length))
   ;; std_msgs/Float64MultiArray _tcp_force
     (send _tcp_force :deserialize buf ptr-) (incf ptr- (send _tcp_force :serialization-length))
   ;; std_msgs/Float64 _time
     (send _time :deserialize buf ptr-) (incf ptr- (send _time :serialization-length))
   ;; std_msgs/Bool _is_steady
     (send _is_steady :deserialize buf ptr-) (incf ptr- (send _is_steady :serialization-length))
   ;;
   self)
  )

(setf (get robotic_deposition::ur5e_data :md5sum-) "273f5ae00ca10a4d616012f38fd5c175")
(setf (get robotic_deposition::ur5e_data :datatype-) "robotic_deposition/ur5e_data")
(setf (get robotic_deposition::ur5e_data :definition-)
      "std_msgs/Float64MultiArray joint_angles
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
# dim[0].label  = \"height\"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = \"width\"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = \"channel\"
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
")



(provide :robotic_deposition/ur5e_data "273f5ae00ca10a4d616012f38fd5c175")



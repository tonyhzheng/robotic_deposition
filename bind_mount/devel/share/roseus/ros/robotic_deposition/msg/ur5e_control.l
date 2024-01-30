;; Auto-generated. Do not edit!


(when (boundp 'robotic_deposition::ur5e_control)
  (if (not (find-package "ROBOTIC_DEPOSITION"))
    (make-package "ROBOTIC_DEPOSITION"))
  (shadow 'ur5e_control (find-package "ROBOTIC_DEPOSITION")))
(unless (find-package "ROBOTIC_DEPOSITION::UR5E_CONTROL")
  (make-package "ROBOTIC_DEPOSITION::UR5E_CONTROL"))

(in-package "ROS")
;;//! \htmlinclude ur5e_control.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass robotic_deposition::ur5e_control
  :super ros::object
  :slots (_inputs _command_type ))

(defmethod robotic_deposition::ur5e_control
  (:init
   (&key
    ((:inputs __inputs) (instance std_msgs::Float64MultiArray :init))
    ((:command_type __command_type) (instance std_msgs::String :init))
    )
   (send-super :init)
   (setq _inputs __inputs)
   (setq _command_type __command_type)
   self)
  (:inputs
   (&rest __inputs)
   (if (keywordp (car __inputs))
       (send* _inputs __inputs)
     (progn
       (if __inputs (setq _inputs (car __inputs)))
       _inputs)))
  (:command_type
   (&rest __command_type)
   (if (keywordp (car __command_type))
       (send* _command_type __command_type)
     (progn
       (if __command_type (setq _command_type (car __command_type)))
       _command_type)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Float64MultiArray _inputs
    (send _inputs :serialization-length)
    ;; std_msgs/String _command_type
    (send _command_type :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Float64MultiArray _inputs
       (send _inputs :serialize s)
     ;; std_msgs/String _command_type
       (send _command_type :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Float64MultiArray _inputs
     (send _inputs :deserialize buf ptr-) (incf ptr- (send _inputs :serialization-length))
   ;; std_msgs/String _command_type
     (send _command_type :deserialize buf ptr-) (incf ptr- (send _command_type :serialization-length))
   ;;
   self)
  )

(setf (get robotic_deposition::ur5e_control :md5sum-) "6af43574263174a907cc0088df6dfae1")
(setf (get robotic_deposition::ur5e_control :datatype-) "robotic_deposition/ur5e_control")
(setf (get robotic_deposition::ur5e_control :definition-)
      "std_msgs/Float64MultiArray inputs
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
MSG: std_msgs/String
string data

")



(provide :robotic_deposition/ur5e_control "6af43574263174a907cc0088df6dfae1")



; Auto-generated. Do not edit!


(cl:in-package robotic_deposition-msg)


;//! \htmlinclude ur5e_data.msg.html

(cl:defclass <ur5e_data> (roslisp-msg-protocol:ros-message)
  ((joint_angles
    :reader joint_angles
    :initarg :joint_angles
    :type std_msgs-msg:Float64MultiArray
    :initform (cl:make-instance 'std_msgs-msg:Float64MultiArray))
   (joint_velocities
    :reader joint_velocities
    :initarg :joint_velocities
    :type std_msgs-msg:Float64MultiArray
    :initform (cl:make-instance 'std_msgs-msg:Float64MultiArray))
   (tcp_pose
    :reader tcp_pose
    :initarg :tcp_pose
    :type std_msgs-msg:Float64MultiArray
    :initform (cl:make-instance 'std_msgs-msg:Float64MultiArray))
   (tcp_speed
    :reader tcp_speed
    :initarg :tcp_speed
    :type std_msgs-msg:Float64MultiArray
    :initform (cl:make-instance 'std_msgs-msg:Float64MultiArray))
   (tcp_force
    :reader tcp_force
    :initarg :tcp_force
    :type std_msgs-msg:Float64MultiArray
    :initform (cl:make-instance 'std_msgs-msg:Float64MultiArray))
   (time
    :reader time
    :initarg :time
    :type std_msgs-msg:Float64
    :initform (cl:make-instance 'std_msgs-msg:Float64))
   (is_steady
    :reader is_steady
    :initarg :is_steady
    :type std_msgs-msg:Bool
    :initform (cl:make-instance 'std_msgs-msg:Bool)))
)

(cl:defclass ur5e_data (<ur5e_data>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ur5e_data>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ur5e_data)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robotic_deposition-msg:<ur5e_data> is deprecated: use robotic_deposition-msg:ur5e_data instead.")))

(cl:ensure-generic-function 'joint_angles-val :lambda-list '(m))
(cl:defmethod joint_angles-val ((m <ur5e_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotic_deposition-msg:joint_angles-val is deprecated.  Use robotic_deposition-msg:joint_angles instead.")
  (joint_angles m))

(cl:ensure-generic-function 'joint_velocities-val :lambda-list '(m))
(cl:defmethod joint_velocities-val ((m <ur5e_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotic_deposition-msg:joint_velocities-val is deprecated.  Use robotic_deposition-msg:joint_velocities instead.")
  (joint_velocities m))

(cl:ensure-generic-function 'tcp_pose-val :lambda-list '(m))
(cl:defmethod tcp_pose-val ((m <ur5e_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotic_deposition-msg:tcp_pose-val is deprecated.  Use robotic_deposition-msg:tcp_pose instead.")
  (tcp_pose m))

(cl:ensure-generic-function 'tcp_speed-val :lambda-list '(m))
(cl:defmethod tcp_speed-val ((m <ur5e_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotic_deposition-msg:tcp_speed-val is deprecated.  Use robotic_deposition-msg:tcp_speed instead.")
  (tcp_speed m))

(cl:ensure-generic-function 'tcp_force-val :lambda-list '(m))
(cl:defmethod tcp_force-val ((m <ur5e_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotic_deposition-msg:tcp_force-val is deprecated.  Use robotic_deposition-msg:tcp_force instead.")
  (tcp_force m))

(cl:ensure-generic-function 'time-val :lambda-list '(m))
(cl:defmethod time-val ((m <ur5e_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotic_deposition-msg:time-val is deprecated.  Use robotic_deposition-msg:time instead.")
  (time m))

(cl:ensure-generic-function 'is_steady-val :lambda-list '(m))
(cl:defmethod is_steady-val ((m <ur5e_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotic_deposition-msg:is_steady-val is deprecated.  Use robotic_deposition-msg:is_steady instead.")
  (is_steady m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ur5e_data>) ostream)
  "Serializes a message object of type '<ur5e_data>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'joint_angles) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'joint_velocities) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'tcp_pose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'tcp_speed) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'tcp_force) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'time) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'is_steady) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ur5e_data>) istream)
  "Deserializes a message object of type '<ur5e_data>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'joint_angles) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'joint_velocities) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'tcp_pose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'tcp_speed) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'tcp_force) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'time) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'is_steady) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ur5e_data>)))
  "Returns string type for a message object of type '<ur5e_data>"
  "robotic_deposition/ur5e_data")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ur5e_data)))
  "Returns string type for a message object of type 'ur5e_data"
  "robotic_deposition/ur5e_data")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ur5e_data>)))
  "Returns md5sum for a message object of type '<ur5e_data>"
  "273f5ae00ca10a4d616012f38fd5c175")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ur5e_data)))
  "Returns md5sum for a message object of type 'ur5e_data"
  "273f5ae00ca10a4d616012f38fd5c175")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ur5e_data>)))
  "Returns full string definition for message of type '<ur5e_data>"
  (cl:format cl:nil "std_msgs/Float64MultiArray joint_angles~%std_msgs/Float64MultiArray joint_velocities~%std_msgs/Float64MultiArray tcp_pose~%std_msgs/Float64MultiArray tcp_speed~%std_msgs/Float64MultiArray tcp_force~%std_msgs/Float64  time~%std_msgs/Bool is_steady~%================================================================================~%MSG: std_msgs/Float64MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%float64[]         data          # array of data~%~%~%================================================================================~%MSG: std_msgs/MultiArrayLayout~%# The multiarray declares a generic multi-dimensional array of a~%# particular data type.  Dimensions are ordered from outer most~%# to inner most.~%~%MultiArrayDimension[] dim # Array of dimension properties~%uint32 data_offset        # padding elements at front of data~%~%# Accessors should ALWAYS be written in terms of dimension stride~%# and specified outer-most dimension first.~%# ~%# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]~%#~%# A standard, 3-channel 640x480 image with interleaved color channels~%# would be specified as:~%#~%# dim[0].label  = \"height\"~%# dim[0].size   = 480~%# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)~%# dim[1].label  = \"width\"~%# dim[1].size   = 640~%# dim[1].stride = 3*640 = 1920~%# dim[2].label  = \"channel\"~%# dim[2].size   = 3~%# dim[2].stride = 3~%#~%# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.~%~%================================================================================~%MSG: std_msgs/MultiArrayDimension~%string label   # label of given dimension~%uint32 size    # size of given dimension (in type units)~%uint32 stride  # stride of given dimension~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: std_msgs/Bool~%bool data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ur5e_data)))
  "Returns full string definition for message of type 'ur5e_data"
  (cl:format cl:nil "std_msgs/Float64MultiArray joint_angles~%std_msgs/Float64MultiArray joint_velocities~%std_msgs/Float64MultiArray tcp_pose~%std_msgs/Float64MultiArray tcp_speed~%std_msgs/Float64MultiArray tcp_force~%std_msgs/Float64  time~%std_msgs/Bool is_steady~%================================================================================~%MSG: std_msgs/Float64MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%float64[]         data          # array of data~%~%~%================================================================================~%MSG: std_msgs/MultiArrayLayout~%# The multiarray declares a generic multi-dimensional array of a~%# particular data type.  Dimensions are ordered from outer most~%# to inner most.~%~%MultiArrayDimension[] dim # Array of dimension properties~%uint32 data_offset        # padding elements at front of data~%~%# Accessors should ALWAYS be written in terms of dimension stride~%# and specified outer-most dimension first.~%# ~%# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]~%#~%# A standard, 3-channel 640x480 image with interleaved color channels~%# would be specified as:~%#~%# dim[0].label  = \"height\"~%# dim[0].size   = 480~%# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)~%# dim[1].label  = \"width\"~%# dim[1].size   = 640~%# dim[1].stride = 3*640 = 1920~%# dim[2].label  = \"channel\"~%# dim[2].size   = 3~%# dim[2].stride = 3~%#~%# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.~%~%================================================================================~%MSG: std_msgs/MultiArrayDimension~%string label   # label of given dimension~%uint32 size    # size of given dimension (in type units)~%uint32 stride  # stride of given dimension~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: std_msgs/Bool~%bool data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ur5e_data>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'joint_angles))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'joint_velocities))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'tcp_pose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'tcp_speed))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'tcp_force))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'time))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'is_steady))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ur5e_data>))
  "Converts a ROS message object to a list"
  (cl:list 'ur5e_data
    (cl:cons ':joint_angles (joint_angles msg))
    (cl:cons ':joint_velocities (joint_velocities msg))
    (cl:cons ':tcp_pose (tcp_pose msg))
    (cl:cons ':tcp_speed (tcp_speed msg))
    (cl:cons ':tcp_force (tcp_force msg))
    (cl:cons ':time (time msg))
    (cl:cons ':is_steady (is_steady msg))
))

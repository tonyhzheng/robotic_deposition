; Auto-generated. Do not edit!


(cl:in-package robotic_deposition-msg)


;//! \htmlinclude ur5e_control.msg.html

(cl:defclass <ur5e_control> (roslisp-msg-protocol:ros-message)
  ((inputs
    :reader inputs
    :initarg :inputs
    :type std_msgs-msg:Float64MultiArray
    :initform (cl:make-instance 'std_msgs-msg:Float64MultiArray))
   (command_type
    :reader command_type
    :initarg :command_type
    :type std_msgs-msg:String
    :initform (cl:make-instance 'std_msgs-msg:String)))
)

(cl:defclass ur5e_control (<ur5e_control>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ur5e_control>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ur5e_control)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robotic_deposition-msg:<ur5e_control> is deprecated: use robotic_deposition-msg:ur5e_control instead.")))

(cl:ensure-generic-function 'inputs-val :lambda-list '(m))
(cl:defmethod inputs-val ((m <ur5e_control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotic_deposition-msg:inputs-val is deprecated.  Use robotic_deposition-msg:inputs instead.")
  (inputs m))

(cl:ensure-generic-function 'command_type-val :lambda-list '(m))
(cl:defmethod command_type-val ((m <ur5e_control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotic_deposition-msg:command_type-val is deprecated.  Use robotic_deposition-msg:command_type instead.")
  (command_type m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ur5e_control>) ostream)
  "Serializes a message object of type '<ur5e_control>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'inputs) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'command_type) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ur5e_control>) istream)
  "Deserializes a message object of type '<ur5e_control>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'inputs) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'command_type) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ur5e_control>)))
  "Returns string type for a message object of type '<ur5e_control>"
  "robotic_deposition/ur5e_control")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ur5e_control)))
  "Returns string type for a message object of type 'ur5e_control"
  "robotic_deposition/ur5e_control")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ur5e_control>)))
  "Returns md5sum for a message object of type '<ur5e_control>"
  "6af43574263174a907cc0088df6dfae1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ur5e_control)))
  "Returns md5sum for a message object of type 'ur5e_control"
  "6af43574263174a907cc0088df6dfae1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ur5e_control>)))
  "Returns full string definition for message of type '<ur5e_control>"
  (cl:format cl:nil "std_msgs/Float64MultiArray inputs~%std_msgs/String  command_type~%================================================================================~%MSG: std_msgs/Float64MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%float64[]         data          # array of data~%~%~%================================================================================~%MSG: std_msgs/MultiArrayLayout~%# The multiarray declares a generic multi-dimensional array of a~%# particular data type.  Dimensions are ordered from outer most~%# to inner most.~%~%MultiArrayDimension[] dim # Array of dimension properties~%uint32 data_offset        # padding elements at front of data~%~%# Accessors should ALWAYS be written in terms of dimension stride~%# and specified outer-most dimension first.~%# ~%# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]~%#~%# A standard, 3-channel 640x480 image with interleaved color channels~%# would be specified as:~%#~%# dim[0].label  = \"height\"~%# dim[0].size   = 480~%# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)~%# dim[1].label  = \"width\"~%# dim[1].size   = 640~%# dim[1].stride = 3*640 = 1920~%# dim[2].label  = \"channel\"~%# dim[2].size   = 3~%# dim[2].stride = 3~%#~%# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.~%~%================================================================================~%MSG: std_msgs/MultiArrayDimension~%string label   # label of given dimension~%uint32 size    # size of given dimension (in type units)~%uint32 stride  # stride of given dimension~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ur5e_control)))
  "Returns full string definition for message of type 'ur5e_control"
  (cl:format cl:nil "std_msgs/Float64MultiArray inputs~%std_msgs/String  command_type~%================================================================================~%MSG: std_msgs/Float64MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%float64[]         data          # array of data~%~%~%================================================================================~%MSG: std_msgs/MultiArrayLayout~%# The multiarray declares a generic multi-dimensional array of a~%# particular data type.  Dimensions are ordered from outer most~%# to inner most.~%~%MultiArrayDimension[] dim # Array of dimension properties~%uint32 data_offset        # padding elements at front of data~%~%# Accessors should ALWAYS be written in terms of dimension stride~%# and specified outer-most dimension first.~%# ~%# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]~%#~%# A standard, 3-channel 640x480 image with interleaved color channels~%# would be specified as:~%#~%# dim[0].label  = \"height\"~%# dim[0].size   = 480~%# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)~%# dim[1].label  = \"width\"~%# dim[1].size   = 640~%# dim[1].stride = 3*640 = 1920~%# dim[2].label  = \"channel\"~%# dim[2].size   = 3~%# dim[2].stride = 3~%#~%# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.~%~%================================================================================~%MSG: std_msgs/MultiArrayDimension~%string label   # label of given dimension~%uint32 size    # size of given dimension (in type units)~%uint32 stride  # stride of given dimension~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ur5e_control>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'inputs))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'command_type))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ur5e_control>))
  "Converts a ROS message object to a list"
  (cl:list 'ur5e_control
    (cl:cons ':inputs (inputs msg))
    (cl:cons ':command_type (command_type msg))
))

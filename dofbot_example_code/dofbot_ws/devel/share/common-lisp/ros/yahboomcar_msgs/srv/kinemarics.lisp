; Auto-generated. Do not edit!


(cl:in-package yahboomcar_msgs-srv)


;//! \htmlinclude kinemarics-request.msg.html

(cl:defclass <kinemarics-request> (roslisp-msg-protocol:ros-message)
  ((kin_name
    :reader kin_name
    :initarg :kin_name
    :type cl:string
    :initform "")
   (src_pos
    :reader src_pos
    :initarg :src_pos
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (src_joints
    :reader src_joints
    :initarg :src_joints
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass kinemarics-request (<kinemarics-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <kinemarics-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'kinemarics-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name yahboomcar_msgs-srv:<kinemarics-request> is deprecated: use yahboomcar_msgs-srv:kinemarics-request instead.")))

(cl:ensure-generic-function 'kin_name-val :lambda-list '(m))
(cl:defmethod kin_name-val ((m <kinemarics-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yahboomcar_msgs-srv:kin_name-val is deprecated.  Use yahboomcar_msgs-srv:kin_name instead.")
  (kin_name m))

(cl:ensure-generic-function 'src_pos-val :lambda-list '(m))
(cl:defmethod src_pos-val ((m <kinemarics-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yahboomcar_msgs-srv:src_pos-val is deprecated.  Use yahboomcar_msgs-srv:src_pos instead.")
  (src_pos m))

(cl:ensure-generic-function 'src_joints-val :lambda-list '(m))
(cl:defmethod src_joints-val ((m <kinemarics-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yahboomcar_msgs-srv:src_joints-val is deprecated.  Use yahboomcar_msgs-srv:src_joints instead.")
  (src_joints m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <kinemarics-request>) ostream)
  "Serializes a message object of type '<kinemarics-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'kin_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'kin_name))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'src_pos))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'src_pos))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'src_joints))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'src_joints))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <kinemarics-request>) istream)
  "Deserializes a message object of type '<kinemarics-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'kin_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'kin_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'src_pos) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'src_pos)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'src_joints) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'src_joints)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<kinemarics-request>)))
  "Returns string type for a service object of type '<kinemarics-request>"
  "yahboomcar_msgs/kinemaricsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'kinemarics-request)))
  "Returns string type for a service object of type 'kinemarics-request"
  "yahboomcar_msgs/kinemaricsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<kinemarics-request>)))
  "Returns md5sum for a message object of type '<kinemarics-request>"
  "b50531ac053c7ba8a5e7dbdab4ee0d01")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'kinemarics-request)))
  "Returns md5sum for a message object of type 'kinemarics-request"
  "b50531ac053c7ba8a5e7dbdab4ee0d01")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<kinemarics-request>)))
  "Returns full string definition for message of type '<kinemarics-request>"
  (cl:format cl:nil "# request~%string  kin_name~%float64[] src_pos~%float64[] src_joints~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'kinemarics-request)))
  "Returns full string definition for message of type 'kinemarics-request"
  (cl:format cl:nil "# request~%string  kin_name~%float64[] src_pos~%float64[] src_joints~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <kinemarics-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'kin_name))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'src_pos) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'src_joints) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <kinemarics-request>))
  "Converts a ROS message object to a list"
  (cl:list 'kinemarics-request
    (cl:cons ':kin_name (kin_name msg))
    (cl:cons ':src_pos (src_pos msg))
    (cl:cons ':src_joints (src_joints msg))
))
;//! \htmlinclude kinemarics-response.msg.html

(cl:defclass <kinemarics-response> (roslisp-msg-protocol:ros-message)
  ((target_joints
    :reader target_joints
    :initarg :target_joints
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (target_pos
    :reader target_pos
    :initarg :target_pos
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass kinemarics-response (<kinemarics-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <kinemarics-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'kinemarics-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name yahboomcar_msgs-srv:<kinemarics-response> is deprecated: use yahboomcar_msgs-srv:kinemarics-response instead.")))

(cl:ensure-generic-function 'target_joints-val :lambda-list '(m))
(cl:defmethod target_joints-val ((m <kinemarics-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yahboomcar_msgs-srv:target_joints-val is deprecated.  Use yahboomcar_msgs-srv:target_joints instead.")
  (target_joints m))

(cl:ensure-generic-function 'target_pos-val :lambda-list '(m))
(cl:defmethod target_pos-val ((m <kinemarics-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yahboomcar_msgs-srv:target_pos-val is deprecated.  Use yahboomcar_msgs-srv:target_pos instead.")
  (target_pos m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <kinemarics-response>) ostream)
  "Serializes a message object of type '<kinemarics-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'target_joints))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'target_joints))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'target_pos))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'target_pos))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <kinemarics-response>) istream)
  "Deserializes a message object of type '<kinemarics-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'target_joints) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'target_joints)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'target_pos) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'target_pos)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<kinemarics-response>)))
  "Returns string type for a service object of type '<kinemarics-response>"
  "yahboomcar_msgs/kinemaricsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'kinemarics-response)))
  "Returns string type for a service object of type 'kinemarics-response"
  "yahboomcar_msgs/kinemaricsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<kinemarics-response>)))
  "Returns md5sum for a message object of type '<kinemarics-response>"
  "b50531ac053c7ba8a5e7dbdab4ee0d01")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'kinemarics-response)))
  "Returns md5sum for a message object of type 'kinemarics-response"
  "b50531ac053c7ba8a5e7dbdab4ee0d01")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<kinemarics-response>)))
  "Returns full string definition for message of type '<kinemarics-response>"
  (cl:format cl:nil "# response~%float64[] target_joints~%float64[] target_pos~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'kinemarics-response)))
  "Returns full string definition for message of type 'kinemarics-response"
  (cl:format cl:nil "# response~%float64[] target_joints~%float64[] target_pos~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <kinemarics-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'target_joints) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'target_pos) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <kinemarics-response>))
  "Converts a ROS message object to a list"
  (cl:list 'kinemarics-response
    (cl:cons ':target_joints (target_joints msg))
    (cl:cons ':target_pos (target_pos msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'kinemarics)))
  'kinemarics-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'kinemarics)))
  'kinemarics-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'kinemarics)))
  "Returns string type for a service object of type '<kinemarics>"
  "yahboomcar_msgs/kinemarics")
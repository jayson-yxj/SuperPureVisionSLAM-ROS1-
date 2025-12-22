
(cl:in-package :asdf)

(defsystem "orb_slam3_ros-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ImagePose" :depends-on ("_package_ImagePose"))
    (:file "_package_ImagePose" :depends-on ("_package"))
  ))
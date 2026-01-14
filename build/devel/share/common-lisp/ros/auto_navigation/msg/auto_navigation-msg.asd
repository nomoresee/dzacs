
(cl:in-package :asdf)

(defsystem "auto_navigation-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "NavigationGoal" :depends-on ("_package_NavigationGoal"))
    (:file "_package_NavigationGoal" :depends-on ("_package"))
    (:file "NavigationStatus" :depends-on ("_package_NavigationStatus"))
    (:file "_package_NavigationStatus" :depends-on ("_package"))
  ))
;; Auto-generated. Do not edit!


(when (boundp 'jetmax_control::ActionSetRawGoal)
  (if (not (find-package "JETMAX_CONTROL"))
    (make-package "JETMAX_CONTROL"))
  (shadow 'ActionSetRawGoal (find-package "JETMAX_CONTROL")))
(unless (find-package "JETMAX_CONTROL::ACTIONSETRAWGOAL")
  (make-package "JETMAX_CONTROL::ACTIONSETRAWGOAL"))

(in-package "ROS")
;;//! \htmlinclude ActionSetRawGoal.msg.html


(defclass jetmax_control::ActionSetRawGoal
  :super ros::object
  :slots (_data _repeat ))

(defmethod jetmax_control::ActionSetRawGoal
  (:init
   (&key
    ((:data __data) "")
    ((:repeat __repeat) 0)
    )
   (send-super :init)
   (setq _data (string __data))
   (setq _repeat (round __repeat))
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:repeat
   (&optional __repeat)
   (if __repeat (setq _repeat __repeat)) _repeat)
  (:serialization-length
   ()
   (+
    ;; string _data
    4 (length _data)
    ;; uint32 _repeat
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _data
       (write-long (length _data) s) (princ _data s)
     ;; uint32 _repeat
       (write-long _repeat s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _data
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _data (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint32 _repeat
     (setq _repeat (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get jetmax_control::ActionSetRawGoal :md5sum-) "58ae1817d1d041c9a34f0f909e7566dc")
(setf (get jetmax_control::ActionSetRawGoal :datatype-) "jetmax_control/ActionSetRawGoal")
(setf (get jetmax_control::ActionSetRawGoal :definition-)
      "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
string data
uint32 repeat

")



(provide :jetmax_control/ActionSetRawGoal "58ae1817d1d041c9a34f0f909e7566dc")


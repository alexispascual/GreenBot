;; Auto-generated. Do not edit!


(when (boundp 'custom_msgs::IntList)
  (if (not (find-package "CUSTOM_MSGS"))
    (make-package "CUSTOM_MSGS"))
  (shadow 'IntList (find-package "CUSTOM_MSGS")))
(unless (find-package "CUSTOM_MSGS::INTLIST")
  (make-package "CUSTOM_MSGS::INTLIST"))

(in-package "ROS")
;;//! \htmlinclude IntList.msg.html


(defclass custom_msgs::IntList
  :super ros::object
  :slots (_data ))

(defmethod custom_msgs::IntList
  (:init
   (&key
    ((:data __data) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _data __data)
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; int32[] _data
    (* 4    (length _data)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32[] _data
     (write-long (length _data) s)
     (dotimes (i (length _data))
       (write-long (elt _data i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32[] _data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _data (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _data i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get custom_msgs::IntList :md5sum-) "563b27884d008b0d2adff54dc1f9e4f5")
(setf (get custom_msgs::IntList :datatype-) "custom_msgs/IntList")
(setf (get custom_msgs::IntList :definition-)
      "int32[] data

")



(provide :custom_msgs/IntList "563b27884d008b0d2adff54dc1f9e4f5")



#!/bin/bash

#sleep 5
# username test_name set_number label
USERNAME="arnthor"
SET=1
rosrun user_tracker recognition_test.py add $USERNAME front $SET
#rosrun user_tracker recognition_test.py add $USERNAME distance $SET 
#rosrun user_tracker recognition_test.py add $USERNAME angle $SET 0
#rosrun user_tracker recognition_test.py add $USERNAME angle $SET 10
#rosrun user_tracker recognition_test.py add $USERNAME angle $SET 20
#rosrun user_tracker recognition_test.py add $USERNAME angle $SET 30
#rosrun user_tracker recognition_test.py add $USERNAME angle $SET 40
#rosrun user_tracker recognition_test.py add $USERNAME angle $SET 50

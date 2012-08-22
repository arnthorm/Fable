#!/bin/bash
rosservice call /add_object "username: '$2' 
user_id: $1"

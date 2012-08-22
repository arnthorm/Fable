#!/bin/bash
rosservice call /add_user "username: '$2' user_id: $1"

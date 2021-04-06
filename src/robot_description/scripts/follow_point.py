#!/usr/bin/python3
"""
Subscribes to:
    /robot/name_of_topic
    this grabs the value (an x,y point in this case) and allows you$ato read 
    it and use it to steer the robot. You can call this by using:
    self.sub_topic = rospy.Subscriber("/robot/name_of_topic", Point)

Publishses to:
    /robot_base_velocity_controller/cmd_vel
    /front_axle_position_controller/command
    this is what you will use to control the robot. By publishing to these topics you will be controlling
    wheel speed and turning angle.
    self.pub_twist = rospy.Publisher("/robot_base_velocity_controller/cmd_vel", Twist, queue_size=5)
"""
# k2_wire_feed
ROS-based wire feeding via Teknic SC-series Clearpath motors

the coding practices are bad, but so it goes

the WireFeedDistanceServer node opens an action server for requesting motion specified by wire feed actions. These actions are defined as
'''
# Goal
int32 axis_number         # the axis to which the request is directed
float64 distance		#desired distance to push the wire (mm)
float64 velocity		#desired velocity (mm/min)
float64 acceleration		#desired acceleration (mm/s^2)
---
# Result
bool success			# whether the operation was successful
string message			# Optional result message
---
# Feedback
float64 current_distance	# Progress of the wire feed (mm)
float64 percent			# Progress of the wire feed (%)
float64 torque_feedback		# Current motor torque (in-oz)
'''
still figuring out how to get the feedback implemented. need to steal the information from the teknic node during processing and pass it to the rclcpp action feedback handling
-jdh

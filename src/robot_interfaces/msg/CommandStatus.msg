# Result of a RobotCommand, published by command_coordinator on /robot_command_status.

uint8 SUCCEEDED=0
uint8 ABORTED=1

std_msgs/Header header
uint32 id           # id of the RobotCommand
uint8 result        # SUCCEEDED, or ABORTED when a newer command replaced it

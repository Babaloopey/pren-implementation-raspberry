# command_coordinator -> microcontroller, /motor_command, sent continuously at the
# command_coordinator rate (20 Hz) so the microcontroller can stop on a missing link (watchdog).
# On the robot serial_uart forwards it over UART; in the simulator virtual_mcu executes it.

std_msgs/Header header
float32 linear      # m/s, + forward
float32 angular     # rad/s, + left
string posture      # STAND, SIT, LIE
string head         # UP, DOWN
string behavior     # '' or BRAV / TRICK while a behaviour runs

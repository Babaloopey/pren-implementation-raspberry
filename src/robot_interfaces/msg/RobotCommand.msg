# High-level command from state_coordinator to command_coordinator (/robot_command).
# command_coordinator answers every command with a CommandStatus carrying the same id
# (MOVE, TURN, POSTURE, HEAD, BEHAVIOR when finished; the others immediately).
#
#   STOP      stop driving
#   MOVE      drive `value` m straight (+ forward, - backward)
#   TURN      turn `value` rad in place (+ left / counter-clockwise)
#   VELOCITY  drive continuously with `linear` m/s and `angular` rad/s until the next motion command
#   POSTURE   take posture `name` (STAND, SIT, LIE)
#   HEAD      move the head `name` (UP, DOWN)
#   BEHAVIOR  perform behaviour `name` (BRAV, TRICK)
#   SIGNAL    sound + LED `name` (ACK = "Ton + LED", BARK = "Bellen")

uint8 STOP=0
uint8 MOVE=1
uint8 TURN=2
uint8 VELOCITY=3
uint8 POSTURE=4
uint8 HEAD=5
uint8 BEHAVIOR=6
uint8 SIGNAL=7

string STAND="STAND"
string SIT="SIT"
string LIE="LIE"
string UP="UP"
string DOWN="DOWN"
string BRAV="BRAV"
string TRICK="TRICK"
string ACK="ACK"
string BARK="BARK"

std_msgs/Header header
uint32 id           # increasing, used to match the CommandStatus
uint8 action        # one of the action constants above
float32 value       # MOVE: m, TURN: rad
float32 linear      # VELOCITY: m/s
float32 angular     # VELOCITY: rad/s
string name         # POSTURE / HEAD / BEHAVIOR / SIGNAL argument

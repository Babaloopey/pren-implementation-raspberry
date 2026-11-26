# Hand gesture recognised by gesture_detector (robot_vision).
# Published once when a gesture becomes stable on /perception/gestures.
#   SITZ  = open palm in front of the chest
#   PLATZ = index finger points to the floor
#   AUF   = index finger points up
#   TRICK = thumbs up (Kunststueck)

string SITZ="SITZ"
string PLATZ="PLATZ"
string AUF="AUF"
string TRICK="TRICK"

std_msgs/Header header  # stamp of the camera image the gesture became stable in
string command          # one of the constants above
float32 confidence      # 0..1, share of frames in the vote window that agree

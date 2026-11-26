# One object found by bone_detector (robot_vision).

string BONE="bone"
string PYLON="pylon"

string class_id         # BONE or PYLON
float32 score           # 0..1, YOLO confidence
float32 bearing         # rad, horizontal angle from the camera axis, + = left, - = right (REP 103)
float32[4] bbox         # x_min, y_min, x_max, y_max in image pixels
bool has_position       # true only if the camera height is configured
geometry_msgs/Point position  # on the floor in camera_link (x forward, y left, z up); valid if has_position

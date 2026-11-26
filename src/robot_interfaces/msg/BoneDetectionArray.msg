# All bones (or pylons) found in one camera image, published every processed image
# (empty list = nothing seen). /perception/bone_position, /perception/pylon_position.

std_msgs/Header header  # stamp of the camera image, frame_id = camera_link
BoneDetection[] detections  # sorted by score, best first

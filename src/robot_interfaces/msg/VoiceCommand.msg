# Spoken command recognised by voice_command_detector (robot_audio).
# Published once per detection on /perception/voice_cmds.

string SUCH="SUCH"
string HIER="HIER"
string BRAV="BRAV"
string SITZ="SITZ"

std_msgs/Header header  # time the word was detected
string command          # one of the constants above
float32 confidence      # 0..1, openWakeWord score of the triggering chunk

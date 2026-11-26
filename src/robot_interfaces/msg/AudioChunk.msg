# A chunk of mono PCM audio. /audio/raw (device rate) and /audio/prepared (16 kHz).

std_msgs/Header header
uint32 sample_rate      # Hz
int16[] data            # mono samples

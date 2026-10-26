# Nested types, unbounded and fixed arrays, a bounded array, defaults and
# constants, plus a field typed from another package.
uint8 STATUS_UNKNOWN=0
uint8 STATUS_OK=1
uint8 STATUS_FAULT=2
int32 MAX_TRACKS=16
string DEFAULT_FRAME=odom

std_msgs/Header header
string child_frame_id "odom"
FxPoint pose
FxPoint[] history
FxPoint[4] corners
float64[<=8] weights
fx_other_msgs/FxTag tag
uint8 status
bool valid true
float32 confidence 0.5

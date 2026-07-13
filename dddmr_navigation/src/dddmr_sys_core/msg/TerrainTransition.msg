uint8 TRANSITION_CONTINUOUS=0
uint8 TRANSITION_STAIR_UP=1
uint8 TRANSITION_STAIR_DOWN=2

std_msgs/Header header
uint64 snapshot_version
string map_hash
int32 staircase_id
uint8 transition_type

geometry_msgs/Pose entry_pose
geometry_msgs/Pose exit_pose
geometry_msgs/Vector3 up_axis

float32 width_m
float32 riser_height_m
float32 tread_depth_m
int32 step_count
float32 confidence
bool allow_up
bool allow_down

# Terrain classes. UNKNOWN is fail-closed whenever terrain gating is enabled.
uint8 TERRAIN_UNKNOWN=0
uint8 TERRAIN_FLAT=1
uint8 TERRAIN_RAMP=2
uint8 TERRAIN_STAIR_TREAD=3
uint8 TERRAIN_STAIR_RISER=4
uint8 TERRAIN_EDGE=5
uint8 TERRAIN_DROP=6

# Navigation-layer traversal states. These do not change the Go2 gait.
uint8 STATE_NORMAL=0
uint8 STATE_STAIR_PRECHECK=1
uint8 STATE_STAIR_APPROACH=2
uint8 STATE_STAIR_ALIGN=3
uint8 STATE_STAIR_COMMITTED=4
uint8 STATE_STAIR_LANDING_VERIFY=5
uint8 STATE_STAIR_FAULT_LATCH=6

std_msgs/Header header
uint64 snapshot_version
uint64 static_ground_generation
string map_hash

uint8 terrain_class
uint8 traversal_state
int32 surface_id
int32 staircase_id
int32 step_index
int32 step_count

float32 slope_rad
float32 longitudinal_slope_rad
float32 cross_slope_rad
float32 roughness_m
float32 support_ratio
float32 confidence
float32 heading_error_rad
float32 lateral_error_m
float32 data_age_sec
float32 body_roll_rad
float32 body_pitch_rad

bool allow_forward
bool allow_reverse
bool entry_valid
bool at_entry
bool on_stair
bool landing_valid
bool full_body_on_landing
bool drop_detected
bool dynamic_obstacle
bool gait_unchanged

uint16 rejection_code
string rejection_reason

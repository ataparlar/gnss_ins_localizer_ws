# ICD Group 1 and Group 401, the difference between the two groups is the reference frame
# in which the data is expressed. Group 1 is Vehicle while Group 401 is Reference.

std_msgs/Header header
TimeDistanceGroup time_distance
LLA lla
NED velocity                                    # meters / second
float64 roll                                    # degrees
float64 pitch                                   # degrees
float64 heading                                 # degrees
float64 wander_angle                            # degrees
float32 track_angle                             # degrees
float32 speed                                   # meters / second
float32 ang_rate_long                           # degrees / second
float32 ang_rate_trans                          # degrees / second
float32 ang_rate_down                           # degrees / second
float32 acc_long                                # meters / second^2
float32 acc_trans                               # meters / second^2
float32 acc_down                                # meters / second^2
uint8 alignment_status

uint8 STATUS_FULL_NAV                   = 0
uint8 STATUS_FINE_ALIGNMENT             = 1
uint8 STATUS_GC_CHI_2                   = 2
uint8 STATUS_PC_CHI_2                   = 3
uint8 STATUS_GC_CHI_1                   = 4
uint8 STATUS_PC_CHI_1                   = 5
uint8 STATUS_COARSE_LEVELING            = 6
uint8 STATUS_INITIAL_SOLUTION_ASSIGNED  = 7
uint8 STATUS_NO_VALID_SOLUTION          = 8

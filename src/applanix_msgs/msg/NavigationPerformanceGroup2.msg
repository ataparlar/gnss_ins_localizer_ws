# ICD Group 2 and Group 402, the difference between the two groups is the reference frame
# in which the data is expressed. Group 2 is Vehicle while Group 402 is Reference.
uint16 GROUP_ID = 2
std_msgs/Header header
TimeDistanceGroup time_distance
NED pos_rms_error                     # meters 
NED vel_rms_error                     # meters / second
float32 attitude_rms_error_roll       # degrees
float32 attitude_rms_error_pitch      # degrees
float32 attitude_rms_error_heading    # degrees
float32 ellipsoid_error_semi_major    # meters
float32 ellipsoid_error_semi_minor    # meters
float32 ellipsoid_error_orientation   # degrees

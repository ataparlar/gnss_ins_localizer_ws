# ICD Group 3

std_msgs/Header header
TimeDistanceGroup time_distance

int8 NAV_STATUS_UNKNOWN             = -1
int8 NAV_STATUS_NO_DATA             = 0
int8 NAV_STATUS_HORZ_COARSE         = 1   # Vertically unconstrained
int8 NAV_STATUS_3D_COARSE           = 2
int8 NAV_STATUS_HORZ_DGPS           = 3   # Vertically unconstrained
int8 NAV_STATUS_3D_DGPS             = 4
int8 NAV_STATUS_FLOAT_RTK           = 5
int8 NAV_STATUS_INTEGER_WIDE_RTK    = 6
int8 NAV_STATUS_INTEGER_NARROW_RTK  = 7
int8 NAV_STATUS_PCODE               = 8
int8 NAV_STATUS_OMNISTAR_HP         = 9
int8 NAV_STATUS_OMNISTAR_HPXP       = 10
int8 NAV_STATUS_OMNISTAR_HPG2       = 11
int8 NAV_STATUS_OMNISTAR_XP         = 12
int8 NAV_STATUS_OMNISTAR_VBS        = 13
int8 NAV_STATUS_OMNISTAR_G2         = 15  # !! Missing 14 is not a mistake
int8 NAV_STATUS_TRIMBLE_RTX         = 16
int8 NAV_STATUS_OMNISTAR_G4         = 17
int8 NAV_STATUS_OMNISTAR_G2PLUS     = 18
int8 NAV_STATUS_OMNISTAR_G4PLUS     = 19

int8 navigation_solution_status
int8 sv_tracked                           # Number of space vehicles tracked
ChannelStatusGroup[] channel_statuses
float32 hdop                              # Horizontal dilution of precision
float32 vdop                              # Vertical dilution of precision
float32 dgps_correction_latency           # Seconds
uint16 dgps_ref_id
uint32 gps_week
float64 gps_utc_time_offset               # Seconds
float32 gnss_nav_msg_latency              # Seconds from the PPS pulse to the start of the GNSS
                                          # navigation data output
float32 geoidal_seperation                # Meters

uint16 GNSS_RECEIVER_NO_RECEIVER   = 0
uint16 GNSS_RECEIVER_TRIMBLE_BD960 = 16
uint16 GNSS_RECEIVER_TRIMBLE_BD982 = 17
uint16 GNSS_RECEIVER_TRIMBLE_BD970 = 18

uint16 gnss_receiver_type
uint32 gnss_status

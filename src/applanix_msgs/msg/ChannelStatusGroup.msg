uint16 sv_prn             # Space Vehicle Pseudo Random Noise code

uint16 TRACKING_STATUS_L1_IDLE        = 0
# 1 - RESERVED
uint16 TRACKING_STATUS_L1_ACQUISITION = 2
uint16 TRACKING_STATUS_L1_CODE_LOCK   = 3
# 4 - RESERVED
uint16 TRACKING_STATUS_L1_PHASE_LOCK  = 5
uint16 TRACKING_STATUS_L2_IDLE        = 6
# 7 - RESERVED
uint16 TRACKING_STATUS_L2_ACQUISITION = 8
uint16 TRACKING_STATUS_L2_CODE_LOCK   = 9
# 10 - RESERVED
uint16 TRACKING_STATUS_L2_PHASE_LOCK  = 11

uint16 tracking_status
float32 sv_azimuth        # Degrees
float32 sv_elevation      # Degrees
float32 sv_l1_snr         # dB
float32 sv_l2_snr         # dB

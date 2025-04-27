# Remote System Status

# Defines the Kalman filter computation mode (see the table 4 below)
# 0 UNINITIALIZED	The Kalman filter is not initialized and the returned data are all invalid.
# 1 VERTICAL_GYRO	The Kalman filter only rely on a vertical reference to compute roll and pitch angles. Heading and navigation data drift freely.
# 2 AHRS			A heading reference is available, the Kalman filter provides full orientation but navigation data drift freely.
# 3 NAV_VELOCITY	The Kalman filter computes orientation and velocity. Position is freely integrated from velocity estimation.
# 4 NAV_POSITION	Nominal mode, the Kalman filter computes all parameters (attitude, velocity, position). Absolute position is provided. 
uint8 ekf_status

# The raw GPS position type
# 0 NO_SOLUTION		No valid solution available.
# 1 UNKNOWN_TYPE	An unknown solution type has been computed.
# 2 SINGLE			Single point solution position.
# 3 PSRDIFF			Standard Pseudorange Differential Solution (DGPS).
# 4 SBAS			SBAS satellite used for differential corrections.
# 5 OMNISTAR		Omnistar VBS Position (L1 sub-meter).
# 6 RTK_FLOAT		Floating RTK ambiguity solution (20 cms RTK).
# 7 RTK_INT			Integer RTK ambiguity solution (2 cms RTK).
# 8 PPP_FLOAT		Precise Point Positioning with float ambiguities
# 9 PPP_INT			Precise Point Positioning with fixed ambiguities
# 10 FIXED			Fixed location solution position
uint8 gps_pos_status
 
# GPS True Heading status.
# Bit 0-5: enum:
# 0 SOL_COMPUTED		A valid solution has been computed.
# 1 INSUFFICIENT_OBS	Not enough valid SV to compute a solution.
# 2 INTERNAL_ERROR		An internal error has occurred.
# 3 HEIGHT_LIMIT		The height limit has been exceeded.
# Bit 6: mask:
# 1 BASELINE_VALID      The baseline length field is filled and valid.
uint16 gps_hdt_status

# CAN STM PING
bool can_stm_status

uint16 op_mode

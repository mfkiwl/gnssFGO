# One single GNSS observation


SatHeader             sat_header

uint8   RAW=0
uint8   RAW_BASESTATION=1
uint8                 obs_type


uint8   SATCORRECTED=2
uint8   DGPS=3
uint8   DUALFREQ=4
uint8   KLOBUCHAR=5
uint8   PPP=6
uint8                 preprossing_type


geometry_msgs/Vector3 sat_pos
geometry_msgs/Vector3 sat_vel

float64               psr
float64               psr_raw
float64               psr_sat_corrected
float64               psr_std              # deviation interpreted using e.g., cn0
float64               psr_std_measured     # deviation provided by the receiver
bool                  psr_valid

float64               dor
float64               dor_raw
float64               dor_sat_corrected
float64               dor_std
float64               dor_std_measured

float64               cp
float64               cp_std
float64               cp_std_measured
bool                  cp_valid

float64               cn0
float64               snr
float64               ele_deg
float64               azi_deg

float64               locktime
uint8                 lli                  # loss of lock indicator equals to the locktime

float64               time_offset_gps

bool                  half_cyc_valid
bool                  sub_half_cyc

uint8                 track_status

uint8 UNKNOWN=0
uint8 LOS=1
uint8 NLOS=2
uint8 MULTIPATH=3
uint8                 integrity_flag


uint32  reserved1   # could be used as glofreq
float64 reserved2


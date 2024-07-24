# One single GNSS observation


SatHeader             sat_header

uint8   RAW=0
uint8   RAW_BASESTATION=1
uint8                 obs_type

float64               psr
float64               psr_std_measured     # deviation provided by the receiver
bool                  psr_valid

float64               dor
float64               dor_std_measured
bool                  dor_valid

float64               cp
float64               cp_std_measured
bool                  cp_valid

float64               cn0
float64               snr

float64               locktime
uint8                 lli                  # loss of lock indicator equals to the locktime

bool                  half_cyc_valid
bool                  sub_half_cyc

uint8                 track_status


uint32  reserved1   # could be used as glofreq
float64 reserved2


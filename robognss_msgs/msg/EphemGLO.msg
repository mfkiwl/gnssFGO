std_msgs/Header   header
GNSSRawHeader     gnss_header

uint8 ROVER=0
uint8 BASESTATION=1
uint8             source

uint32            sv_id
int32             freq_nr
uint8             sat_type

float64           dt_gps   # seconds between GPS and GLONASS time, >0: GLONASS is ahead of GPS

uint32            health
uint32            iode

geometry_msgs/Vector3 pos    # x coordinate in PZ-90.02 in m
geometry_msgs/Vector3 dpos   # m/s
geometry_msgs/Vector3 ddpos  # lunisolar acceleration m/s^2

float64           tau_n  # correction to the nth satellite time t_n relative to GLONASS time t_c in sec

float64           d_tau_n  # time difference between navigation RF signal transmitted in L2 sub-hand and navigation RF signal transmitted in L1 sub-band by nth satellite in sec

float64           gamma  # frequency correction secs/sec

float64           t_k  # time of frame start since start of GLONASS day in sec
uint32            p  # technological parameter
uint32            f_t  # user range
uint32            age  # age of data in days

uint16            t_b  # time of day in min
uint8             p1  # time interval between adjacent values of t_b
uint8             p2  # 1 bit odd/even flag of t_b

uint8             ura_index

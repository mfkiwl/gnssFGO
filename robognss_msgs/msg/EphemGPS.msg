std_msgs/Header   header
GNSSRawHeader     gnss_header

uint8 ROVER=0
uint8 BASESTATION=1
uint8             source

uint32            sv_id
uint32            health
uint32            iode1
uint32            iode2
uint8             code
uint8             flag
uint32            z_week
float64           t_oe
float64           sqrt_a
float64           delta_n
float64           m_0
float64           ecc
float64           omega
float64           cuc
float64           cus
float64           crc
float64           crs
float64           cic
float64           cis
float64           i_0
float64           i_dot
float64           omega_0
float64           omega_dot
uint32            iodc
float64           t_oc
float64           t_gd
float64           a_f0
float64           a_f1
float64           a_f2
uint32            a_s
float64           n            # Corrected mean motion (radians/second)
float64           ura          # some receivers provide interpreted ura variance
uint8             ura_index    # used in rtklib
uint8             fit_interval_flag
uint16            wnt_oe  
uint16            wnt_oc


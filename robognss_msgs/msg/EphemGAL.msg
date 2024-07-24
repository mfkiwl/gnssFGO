std_msgs/Header   header
GNSSRawHeader     gnss_header

uint8 ROVER=0
uint8 BASESTATION=1
uint8             source

uint32            sv_id

uint8             E1B = 1
uint8             E5A = 2
uint8             E5B = 3
uint8             nav_source

float64           sqrt_a
float64           m_0
float64           ecc
float64           i_0
float64           omega
float64           omega_0
float64           omega_dot
float64           i_dot
float64           delta_n
float64           cuc
float64           cus
float64           crc
float64           crs
float64           cic
float64           cis
float64           a_f0
float64           a_f1
float64           a_f2
float64           t_oe
float64           t_oc
uint16            wnt_oe  
uint16            wnt_oc
uint8             e5_health
uint8             e5_dvs             
uint8             e1b_health
uint8             e1b_dvs
uint16            io_dnav
float64           e1e5a_bgd
float64           e1e5b_bgd
float64           sv_clk_bias
float64           sv_clc_drift
float64           sis_accuracy
uint8             ura_index
std_msgs/Header   header
GNSSRawHeader     gnss_header
SatHeader         sat_header
# ion
float64           a0    # ai0 for Galileo
float64           a1    # ai1 for Galileo
float64           a2    # ai2 for Galileo
float64           a3
float64           b0
float64           b1
float64           b2
float64           b3

# utc
float64           capital_a0
float64           capital_a1
float64           t_ot
float64           wn_t
float64           dt_ls
float64           wn_lsf
float64           dn
float64           dt_lsf

float64           tau_c    # used for GLONASS
float64           tau_gps  # used for GLONASS

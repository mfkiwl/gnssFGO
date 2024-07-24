# GNSS PVT solution in the WGS84 frame 

std_msgs/Header       header
GNSSHeader            gnss_header

float64               lat_deg
float64               lon_deg
float64               height
float64               undulation
geometry_msgs/Vector3 pos_std

Velocity             vel

float64               rx_clk_bias
float64               rx_clk_bias_std
float64               rx_clk_drift
float64               rx_clk_drift_std

float64               pdop

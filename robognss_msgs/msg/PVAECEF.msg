# GNSS PVT solution in the ECEC frame 

std_msgs/Header       header
GNSSHeader            gnss_header

geometry_msgs/Point   pos
geometry_msgs/Vector3 pos_std

Velocity              vel
Attitude              att

float64               pdop

float64               rx_clk_bias
float64               rx_clk_bias_std
float64               rx_clk_drift
float64               rx_clk_drift_std

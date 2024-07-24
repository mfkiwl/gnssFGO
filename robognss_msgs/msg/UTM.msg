# GNSS solution in UTM frame 

std_msgs/Header       header
GNSSHeader            gnss_header

string                utm_zone

float64               easting
float64               northing
float64               height

float64               easting_std
float64               northing_std
float64               height_std
float64               northing_easting_std
float64               northing_height_std
float64               easting_height_std

Velocity              vel
Attitude              att

float64               pdop
float64               reliability_h  # Horizontal position external reliability (m)
float64               reliability_v  # Vertical position external reliability (m)
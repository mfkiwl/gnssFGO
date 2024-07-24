std_msgs/Header    header
StationParam       station

float64            rover_lat
float64            rover_lon
float64            rover_height

float64            last_rtcm_timestamp
float64            last_gga_update_timestamp
float64            last_ntrip_update_timestamp

bool               is_ntrip_publishing

string             ntrip_server
uint32             ntrip_port
string             ntrip_mountpoint
string             ntrip_user
string             ntrip_password


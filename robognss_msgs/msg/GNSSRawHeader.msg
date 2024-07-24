# General GNSS RAW measurement Information

float64   tow   # in sec
uint16    wnc

######### optional gps time in detail
uint16    year # year of GPS time
uint16    doy  # day of the year
float64   tod  # time of the day
float64   gps_time  # gps time since 1980-01-06 00:00:00 UTC
float64   unix_time # converted gps time to unix since 1970-01-01 00:00:00 UTC

string   receiver_name
string   reserved
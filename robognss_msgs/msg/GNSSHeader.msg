# General GNSS Solution Information
#########
uint8     SINGLE = 1
uint8     RTK_FIXED = 2
uint8     RTK_FLOAT = 3
uint8     PPP = 4
uint8     mode  # GNSS mode / type, one the the flags above

#########
uint8     SOLUTION_OK = 1
uint8     SOLUTION_INVALID = 2
uint8     NO_SOLUTION = 0
uint8     sol_status  # solution status, one of the flags above

#########
float64   sol_age     # aging of the solution in sec

bool      diff_correction_applied
float64   diff_age    # aging of the differential correction data in sec

bool      heading_valid
bool      valid_datetime

float64   tow  # in sec
uint16    wnc

######### optional gps time in detail
uint16    year # year of GPS time
uint16    doy  # day of the year
float64   tod  # time of the day
float64   gps_time  # gps time since 1980-01-06 00:00:00 UTC
float64   unix_time # converted gps time to unix since 1970-01-01 00:00:00 UTC

#########
uint8    num_sat_all
uint8    num_sat_used
uint8    num_sat_l1e1
uint8    num_sat_multi

string   receiver_name
string   reserved
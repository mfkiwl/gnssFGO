float64              tow   # in sec
uint8                sat_sys         # SatHeader.GPS
uint16               rtcm_identity   # not used for rinex
uint8                num_sat
uint8                num_code        # num of received freq
uint8                num_cell        # not used for rinex max. of num_cell = num_sat * num_code
Obs[]                obs
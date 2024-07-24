std_msgs/Header       header

uint32                ref_station_id
uint32                phy_ref_station_id

string                marker_name
string                marker_number
string                ant_descriptor
string                ant_serial_number
string                rev_type_descriptor
string                rev_firmware_version
string                rev_serial_number
uint8                 ant_setup_id
uint8                 itrf_realization_year
uint8                 deltype

geometry_msgs/Vector3 station_pos 
geometry_msgs/Vector3 ant_pos 
geometry_msgs/Vector3 ant_position_delta

float64               ant_height
bool                  glo_cp_aligned
float64[4]            glo_cp_bias
uint8                 glo_fdma_signals_mask

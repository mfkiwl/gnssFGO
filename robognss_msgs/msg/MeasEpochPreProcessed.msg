# Epoch of GNSS measurements containing all observations

std_msgs/Header        header
GNSSRawHeader          gnss_header
ObsExtended[]          obs_gps_vec
ObsExtended[]          obs_gal_vec
ObsExtended[]          obs_glo_vec
ObsExtended[]          obs_bds_vec
ObsExtended[]          obs_sbas_vec
ObsExtended[]          obs_qzss_vec
uint8                  ref_sat_svid_gps
uint8                  ref_sat_svid_gal
uint8                  ref_sat_svid_bds
uint8                  ref_sat_svid_glo
uint8                  dd_idx_sync_ref
uint8                  dd_idx_sync_user
geometry_msgs/Vector3  base_pos
uint8                  ant_id
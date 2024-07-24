std_msgs/Header   header
GNSSRawHeader     gnss_header
uint32            sv_id

uint8 ROVER=0
uint8 BASESTATION=1
uint8             source

uint32            health
uint32            iodn
float64           ura
geometry_msgs/Vector3 pos
geometry_msgs/Vector3 dpos
geometry_msgs/Vector3 ddpos
float64           a_f0
float64           a_f1
uint8             ura_index

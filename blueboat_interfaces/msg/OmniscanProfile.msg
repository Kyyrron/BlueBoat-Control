# Raw side-scan sonar profile from one Cerulean Omniscan 450 transducer.
# This is a 1-to-1 mapping of the os_mono_profile packet (id 2198) returned
# by the Cerulean Ping Protocol, with no preprocessing applied.
#
# The amplitude data lives in pwr_results, scaled as u16 between min_pwr_db
# and max_pwr_db. Use Omniscan450.scale_power(...) downstream to recover dB.

std_msgs/Header header

# Which side of the boat this scan came from: "port" or "starboard".
string side

# --- Ping metadata --------------------------------------------------------
uint32 ping_number              # sequential counter, reset at device power-up
uint32 start_mm                 # start of profile range
uint32 length_mm                # length of profile (end_mm = start_mm + length_mm)
uint32 timestamp_ms             # device-side timestamp, msec since power-up
uint32 ping_hz                  # acoustic frequency in Hz (450000 nominal)
uint16 gain_index               # 0-7
uint16 num_results              # length of pwr_results
uint16 sos_dmps                 # speed of sound in decimeters/sec
uint8  channel_number

float32 pulse_duration_sec
float32 analog_gain
float32 max_pwr_db
float32 min_pwr_db
float32 transducer_heading_deg
float32 vehicle_heading_deg

# --- Raw acoustic profile -------------------------------------------------
# u16 power samples, scaled linearly between min_pwr_db and max_pwr_db.
uint16[] pwr_results

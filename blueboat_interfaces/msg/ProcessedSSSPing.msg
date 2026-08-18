# Merged side-scan sonar ping after slant-range correction.
#
# Produced by sss_processor_node from a port + starboard OmniscanProfile pair.
# The processor:
#   - converts raw u16 power samples to dB
#   - estimates transducer altitude above seabed via First-Bottom-Return (FBR)
#     detection across pings (10-ping bootstrap before first publish)
#   - applies slant-range correction, assuming flat seabed locally to each ping
#     (altitude is allowed to vary from ping to ping — non-flat across pings)
#   - drops water-column samples (slant range < altitude)
#   - tags every remaining sample with its y-coordinate in the robot frame at
#     ping time
#
# The robot pose at ping time is snapped to the nearest /blueboat/odom sample.
# The frame label is "base_link" for documentation only — no TF is published
# in this project; downstream consumers read pose via /blueboat/odom directly.

# Traceability — source ping stamps and counters per side.
builtin_interfaces/Time port_stamp
builtin_interfaces/Time starboard_stamp
uint32 port_ping_number
uint32 starboard_ping_number

# --- Robot state at ping time -----------------------------------------------
# Origin-relative pose, snapshotted from /blueboat/odom at the matched-pair
# stamp (nearest-sample lookup). x/y in meters, origin = boot-time anchor.
float32 robot_x
float32 robot_y
geometry_msgs/Quaternion robot_orientation

# --- Bathymetry -------------------------------------------------------------
# Water depth below the robot at ping time, in meters. Computed as the
# (slant) altitude of the transducer above the seabed (from FBR) plus the
# constant transducer submersion offset.
float32 water_depth

# --- Geometry constant ------------------------------------------------------
# Transducer forward offset from base_link origin, in meters. Same on both
# sides (port + starboard transducers are mounted symmetrically along x).
# Published per-message so downstream consumers don't need to know the
# processor's internal constants.
float32 transducer_x_offset

# --- Per-side processed samples --------------------------------------------
# Arrays are NOT the same length on both sides: each side independently
# drops its water-column samples based on the single fused altitude.
#
# Sign convention (REP-103, +y = port = left):
#   port_y[i]      > 0  (port  is +y)
#   starboard_y[i] < 0  (starboard is -y)
# Lateral offsets of each transducer are already folded into the y values,
# so port_y[i] is directly the y-coordinate of sample i in base_link.
float32[] port_intensity_db
float32[] port_y
float32[] starboard_intensity_db
float32[] starboard_y

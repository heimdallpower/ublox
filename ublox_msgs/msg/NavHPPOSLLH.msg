# NAV-HPPOSLLH (0x01 0x14)
# High Precision Geodetic Position Solution
#
# See important comments concerning validity of position given in section
# Navigation Output Filters.
# This message outputs the Geodetic position in the currently selected
# Ellipsoid. The default is the WGS84 Ellipsoid, but can be changed with the
# message CFG-DAT.
#

uint8 CLASS_ID = 1
uint8 MESSAGE_ID = 20

uint8 version
uint8[2] reserved1
int8 invalid_llh

uint32 i_tow    # GPS Millisecond Time of Week [ms]

int32 lon       # Longitude [deg / 1e-7]
int32 lat       # Latitude [deg / 1e-7]
int32 height    # Height above Ellipsoid [mm]
int32 h_msl     # Height above mean sea level [mm]
int8 lon_hp     # Longitude [deg / 1e-9, range -99 to +99]
int8 lat_hp     # Latitude [deg / 1e-9, range -99 to +99]
int8 height_hp  # Height above Ellipsoid [mm / 0.1, range -9 to +9]
int8 h_msl_hp   # Height above mean sea level [mm / 0.1, range -9 to +9]
uint32 h_acc    # Horizontal Accuracy Estimate [mm / 0.1]
uint32 v_acc    # Vertical Accuracy Estimate [mm / 0.1]

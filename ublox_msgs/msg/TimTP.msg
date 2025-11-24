# TIM-TP (0x0D, 0x01)
# Time Pulse Timedata

uint8 CLASS_ID = 13 
uint8 MESSAGE_ID = 1

uint32 tow_ms     # Time pulse time of week according to time base
uint32 tow_sub_ms # Submillisecond part of TOWMS
int32 q_err       # Quantization error of time pulse (not
                  # supported for the FTS product variant).
uint16 week       # Time pulse week number according to time base
uint8 flags       # bitmask
uint8 FLAGS_TIMEBASE  = 1
uint8 FLAGS_UTC       = 2
uint8 FLAGS_RAIM      = 12

uint8 ref_info     # Time reference information
uint8 REFINFO_TIMEREFGNSS = 15
uint8 REFINFO_UTCSTANDARD = 240

std_msgs/Header header                                      # Stamp is ros::Time::now of reception (depends on system load)

uint32          iTOW                                        # GPS Millisecond time of week [ms] for easy synchronizing with other ublox_msgs types

bool            ublox_utc_time_valid                        # The 'ublox_utc_time' is a function the ubx-nax-pvt time data.
                                                            # If that isn't valid, neither is the derived value

time            ublox_utc_time                              # Conversion of NavPVT time data to ros::Time (represents time of measurement)


bool            ublox_utc_to_ros_aligned_time_offset_valid  # Can 'ublox_utc_to_ros_aligned_time_offset' be used to compute system ros::Time of reception?
duration        ublox_utc_to_ros_aligned_time_offset        # System ros::Time of reception computed as ublox_utc_time + ublox_utc_to_ros_aligned_time_offset

std_msgs/Header header
uint32          iTOW                    # GPS Millisecond time of week [ms] for easy synchronizing with other ublox_msgs types
bool            is_time_of_measurement  # Does the 'stamp' field in the header represent the time of measurement or the time (ros::Time::now()) of arrival

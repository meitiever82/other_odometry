# Four-wheel independent steering chassis status feedback (matches deployed device).
# CDR wire format verified against /robot/wheel_status in
# Datasets/w2/rosbag2_2026_04_28-17_06_53 — 76-byte payload, 8 doubles after stamp.

builtin_interfaces/Time stamp

float64 front_left_angle
float64 front_right_angle
float64 rear_left_angle
float64 rear_right_angle

float64 front_left_speed
float64 front_right_speed
float64 rear_left_speed
float64 rear_right_speed

uint64 timestamp              # time since system start (microseconds)
uint64 timestamp_sample       # timestamp of the source sample (microseconds)

bool valid                    # true when compensation terms should be consumed

float32[3] angular_accel_comp # additive angular acceleration compensation [rad/s^2]
float32[3] torque_comp        # additive physical torque compensation [N*m]

# TOPICS rate_ctrl_compensation

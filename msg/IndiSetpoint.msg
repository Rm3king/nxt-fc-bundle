uint64 timestamp              # time since system start (microseconds)
uint64 timestamp_sample       # timestamp of the sample this setpoint is based on (microseconds)

bool valid                    # true when this setpoint is intended to be consumed

uint8 ACCEL_FRAME_NED = 0     # accel_sp is in world NED frame
uint8 ACCEL_FRAME_BODY_FRD = 1 # accel_sp is in body FRD frame
uint8 accel_frame             # frame selector for accel_sp

float32[3] accel_sp           # desired translational acceleration [m/s^2]
float32 heading_sp            # desired yaw/heading in NED frame [rad]
float32[3] body_rate_ff       # body-rate feedforward in body FRD frame [rad/s]
float32[3] body_ang_accel_ff  # body angular acceleration feedforward [rad/s^2]

float32 thrust_min            # minimum normalized collective thrust magnitude [0, 1], NAN to use parameter/default
float32 thrust_max            # maximum normalized collective thrust magnitude [0, 1], NAN to use parameter/default

# TOPICS indi_setpoint

uint64 timestamp                 # time since system start [us]
uint64 timestamp_sample          # gyro sample timestamp [us]

uint8 STATE_IDLE = 0
uint8 STATE_SETTLE = 1
uint8 STATE_ROLL = 2
uint8 STATE_ROLL_PAUSE = 3
uint8 STATE_PITCH = 4
uint8 STATE_PITCH_PAUSE = 5
uint8 STATE_YAW = 6
uint8 STATE_COMPLETE = 7
uint8 STATE_ABORT = 8
uint8 state

uint8 AXIS_NONE = 0
uint8 AXIS_ROLL = 1
uint8 AXIS_PITCH = 2
uint8 AXIS_YAW = 3
uint8 axis

uint8 ABORT_NONE = 0
uint8 ABORT_SWITCH = 1
uint8 ABORT_NOT_PID = 2
uint8 ABORT_NOT_FLYING = 3
uint8 ABORT_RC_LOSS = 4
uint8 ABORT_STICK_INPUT = 5
uint8 ABORT_RATE = 6
uint8 ABORT_RATE_ERROR = 7
uint8 ABORT_TORQUE = 8
uint8 ABORT_ALLOCATION = 9
uint8 ABORT_MODE = 10
uint8 ABORT_ATTITUDE = 11
uint8 abort_reason

bool active
uint8 EXCITATION_RATE_CHIRP = 0
uint8 EXCITATION_TORQUE_CHIRP = 1
uint8 EXCITATION_TORQUE_MULTISINE = 2
uint8 excitation_mode
float32 elapsed_s
float32 frequency_hz
float32 injection
float32[3] rate_injection       # additive body-rate injection before PID [rad/s]
float32[3] torque_injection     # additive normalized torque after PID
float32[3] base_rate_sp          # rate setpoint before identification injection [rad/s]
float32[3] rate_sp               # rate setpoint consumed by PID [rad/s]
float32[3] measured_rate         # measured body rate [rad/s]
float32[3] torque_sp_pid         # normalized stock PID torque before injection
float32[3] torque_sp             # final normalized torque sent to allocator

# TOPICS rate_sysid_status

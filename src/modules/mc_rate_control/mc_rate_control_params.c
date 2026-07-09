/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mc_rate_control_params.c
 *
 * Parameters for multicopter rate controller
 */

/**
 * Roll rate P gain
 *
 * Roll rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @min 0.01
 * @max 0.5
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ROLLRATE_P, 0.15f);

/**
 * Roll rate I gain
 *
 * Roll rate integral gain. Can be set to compensate static thrust difference or gravity center offset.
 *
 * @min 0.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ROLLRATE_I, 0.2f);

/**
 * Roll rate integrator limit
 *
 * Roll rate integrator limit. Can be set to increase the amount of integrator available to counteract disturbances or reduced to improve settling time after large roll moment trim changes.
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_RR_INT_LIM, 0.30f);

/**
 * Roll rate D gain
 *
 * Roll rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @max 0.01
 * @decimal 4
 * @increment 0.0005
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ROLLRATE_D, 0.003f);

/**
 * Roll rate feedforward
 *
 * Improves tracking performance.
 *
 * @min 0.0
 * @decimal 4
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ROLLRATE_FF, 0.0f);

/**
 * Roll rate controller gain
 *
 * Global gain of the controller.
 *
 * This gain scales the P, I and D terms of the controller:
 * output = MC_ROLLRATE_K * (MC_ROLLRATE_P * error
 * 			     + MC_ROLLRATE_I * error_integral
 * 			     + MC_ROLLRATE_D * error_derivative)
 * Set MC_ROLLRATE_P=1 to implement a PID in the ideal form.
 * Set MC_ROLLRATE_K=1 to implement a PID in the parallel form.
 *
 * @min 0.01
 * @max 5.0
 * @decimal 4
 * @increment 0.0005
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ROLLRATE_K, 1.0f);

/**
 * Pitch rate P gain
 *
 * Pitch rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @min 0.01
 * @max 0.6
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_PITCHRATE_P, 0.15f);

/**
 * Pitch rate I gain
 *
 * Pitch rate integral gain. Can be set to compensate static thrust difference or gravity center offset.
 *
 * @min 0.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_PITCHRATE_I, 0.2f);

/**
 * Pitch rate integrator limit
 *
 * Pitch rate integrator limit. Can be set to increase the amount of integrator available to counteract disturbances or reduced to improve settling time after large pitch moment trim changes.
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_PR_INT_LIM, 0.30f);

/**
 * Pitch rate D gain
 *
 * Pitch rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @decimal 4
 * @increment 0.0005
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_PITCHRATE_D, 0.003f);

/**
 * Pitch rate feedforward
 *
 * Improves tracking performance.
 *
 * @min 0.0
 * @decimal 4
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_PITCHRATE_FF, 0.0f);

/**
 * Pitch rate controller gain
 *
 * Global gain of the controller.
 *
 * This gain scales the P, I and D terms of the controller:
 * output = MC_PITCHRATE_K * (MC_PITCHRATE_P * error
 * 			     + MC_PITCHRATE_I * error_integral
 * 			     + MC_PITCHRATE_D * error_derivative)
 * Set MC_PITCHRATE_P=1 to implement a PID in the ideal form.
 * Set MC_PITCHRATE_K=1 to implement a PID in the parallel form.
 *
 * @min 0.01
 * @max 5.0
 * @decimal 4
 * @increment 0.0005
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_PITCHRATE_K, 1.0f);

/**
 * Yaw rate P gain
 *
 * Yaw rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @min 0.0
 * @max 0.6
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRATE_P, 0.2f);

/**
 * Yaw rate I gain
 *
 * Yaw rate integral gain. Can be set to compensate static thrust difference or gravity center offset.
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRATE_I, 0.1f);

/**
 * Yaw rate integrator limit
 *
 * Yaw rate integrator limit. Can be set to increase the amount of integrator available to counteract disturbances or reduced to improve settling time after large yaw moment trim changes.
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_YR_INT_LIM, 0.30f);

/**
 * Yaw rate D gain
 *
 * Yaw rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRATE_D, 0.0f);

/**
 * Yaw rate feedforward
 *
 * Improves tracking performance.
 *
 * @min 0.0
 * @decimal 4
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRATE_FF, 0.0f);

/**
 * Yaw rate controller gain
 *
 * Global gain of the controller.
 *
 * This gain scales the P, I and D terms of the controller:
 * output = MC_YAWRATE_K * (MC_YAWRATE_P * error
 * 			     + MC_YAWRATE_I * error_integral
 * 			     + MC_YAWRATE_D * error_derivative)
 * Set MC_YAWRATE_P=1 to implement a PID in the ideal form.
 * Set MC_YAWRATE_K=1 to implement a PID in the parallel form.
 *
 * @min 0.0
 * @max 5.0
 * @decimal 4
 * @increment 0.0005
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRATE_K, 1.0f);

/**
 * Battery power level scaler
 *
 * This compensates for voltage drop of the battery over time by attempting to
 * normalize performance across the operating range of the battery. The copter
 * should constantly behave as if it was fully charged with reduced max acceleration
 * at lower battery percentages. i.e. if hover is at 0.5 throttle at 100% battery,
 * it will still be 0.5 at 60% battery.
 *
 * @boolean
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_INT32(MC_BAT_SCALE_EN, 0);

/**
 * Multicopter rate controller mode
 *
 * Selects the low-level multicopter rate controller. The default keeps the
 * stock PX4 PID controller unchanged.
 *
 * @value 0 PID
 * @value 1 INDI
 * @min 0
 * @max 1
 * @group Multicopter INDI Control
 */
PARAM_DEFINE_INT32(MC_INDI_MODE, 0);

/**
 * Restrict INDI to Offboard mode
 *
 * When enabled, INDI is only allowed while offboard control is active. Outside
 * Offboard the controller falls back to the stock PID path.
 *
 * @boolean
 * @group Multicopter INDI Control
 */
PARAM_DEFINE_INT32(MC_INDI_ONLY_OF, 1);

/**
 * INDI setpoint adapter mode
 *
 * Selects the setpoint frontend feeding the shared physical INDI core.
 *
 * @value 0 Acceleration + heading + body-rate feedforward
 * @value 1 Collective thrust + body-rate compatibility path
 * @min 0
 * @max 1
 * @group Multicopter INDI Control
 */
PARAM_DEFINE_INT32(MC_INDI_SP_MODE, 0);

/**
 * INDI setpoint timeout
 *
 * Timeout for the accel-heading INDI setpoint stream. When stale, INDI falls
 * back to PID instead of latching the last setpoint.
 *
 * @unit ms
 * @min 10
 * @max 1000
 * @group Multicopter INDI Control
 */
PARAM_DEFINE_INT32(MC_INDI_SP_TO, 100);

/**
 * Vehicle mass for INDI accel-heading frontend
 *
 * Used for logging and future physical thrust conversion. The current PX4
 * normalized thrust adapter uses hover thrust for the final normalized command.
 *
 * @unit kg
 * @min 0.05
 * @max 20.0
 * @decimal 3
 * @group Multicopter INDI Control
 */
PARAM_DEFINE_FLOAT(MC_INDI_MASS, 1.0f);

/**
 * Hover thrust for INDI accel-heading frontend
 *
 * Normalized collective thrust magnitude that approximately hovers the vehicle.
 *
 * @min 0.05
 * @max 0.95
 * @decimal 3
 * @group Multicopter INDI Control
 */
PARAM_DEFINE_FLOAT(MC_INDI_HOV_THR, 0.5f);

/**
 * INDI synchronized filter cutoff
 *
 * First-order low-pass cutoff used identically in mc_rate_control for
 * locally differentiated body-rate angular acceleration and applied-torque
 * estimate. This keeps omega_dot and tau0 on the same controller frame and
 * filter coefficient.
 *
 * @unit Hz
 * @min 1
 * @max 200
 * @decimal 1
 * @group Multicopter INDI Control
 */
PARAM_DEFINE_FLOAT(MC_INDI_FILT, 40.0f);

/**
 * INDI roll inertia
 *
 * Physical or equivalent roll inertia used by the INDI core.
 *
 * @unit kg m^2
 * @min 0.00001
 * @max 1.0
 * @decimal 5
 * @group Multicopter INDI Control
 */
PARAM_DEFINE_FLOAT(MC_INDI_JX, 0.0020f);

/**
 * INDI pitch inertia
 *
 * Physical or equivalent pitch inertia used by the INDI core.
 *
 * @unit kg m^2
 * @min 0.00001
 * @max 1.0
 * @decimal 5
 * @group Multicopter INDI Control
 */
PARAM_DEFINE_FLOAT(MC_INDI_JY, 0.0020f);

/**
 * INDI yaw inertia
 *
 * Physical or equivalent yaw inertia used by the INDI core.
 *
 * @unit kg m^2
 * @min 0.00001
 * @max 1.0
 * @decimal 5
 * @group Multicopter INDI Control
 */
PARAM_DEFINE_FLOAT(MC_INDI_JZ, 0.0040f);

/**
 * INDI roll torque normalization scale
 *
 * Physical roll torque that maps to normalized torque command 1.0.
 *
 * @unit Nm
 * @min 0.0001
 * @max 10.0
 * @decimal 4
 * @group Multicopter INDI Control
 */
PARAM_DEFINE_FLOAT(MC_INDI_TMAX_X, 0.10f);

/**
 * INDI pitch torque normalization scale
 *
 * Physical pitch torque that maps to normalized torque command 1.0.
 *
 * @unit Nm
 * @min 0.0001
 * @max 10.0
 * @decimal 4
 * @group Multicopter INDI Control
 */
PARAM_DEFINE_FLOAT(MC_INDI_TMAX_Y, 0.10f);

/**
 * INDI yaw torque normalization scale
 *
 * Physical yaw torque that maps to normalized torque command 1.0.
 *
 * @unit Nm
 * @min 0.0001
 * @max 10.0
 * @decimal 4
 * @group Multicopter INDI Control
 */
PARAM_DEFINE_FLOAT(MC_INDI_TMAX_Z, 0.03f);

/**
 * INDI roll attitude frontend gain
 *
 * Maps accel-heading attitude error to body-rate setpoint.
 *
 * @unit 1/s
 * @min 0
 * @max 20
 * @decimal 2
 * @group Multicopter INDI Control
 */
PARAM_DEFINE_FLOAT(MC_INDI_AKP_X, 4.0f);

/**
 * INDI pitch attitude frontend gain
 *
 * Maps accel-heading attitude error to body-rate setpoint.
 *
 * @unit 1/s
 * @min 0
 * @max 20
 * @decimal 2
 * @group Multicopter INDI Control
 */
PARAM_DEFINE_FLOAT(MC_INDI_AKP_Y, 4.0f);

/**
 * INDI yaw attitude frontend gain
 *
 * Maps accel-heading attitude error to body-rate setpoint.
 *
 * @unit 1/s
 * @min 0
 * @max 20
 * @decimal 2
 * @group Multicopter INDI Control
 */
PARAM_DEFINE_FLOAT(MC_INDI_AKP_Z, 2.0f);

/**
 * INDI attitude yaw weight
 *
 * Yaw deprioritization weight for the accel-heading frontend.
 *
 * @min 0
 * @max 1
 * @decimal 2
 * @group Multicopter INDI Control
 */
PARAM_DEFINE_FLOAT(MC_INDI_YAW_W, 0.4f);

/**
 * INDI roll rate-to-acceleration gain
 *
 * Maps body-rate error to desired roll angular acceleration.
 *
 * @unit 1/s
 * @min 0
 * @max 200
 * @decimal 2
 * @group Multicopter INDI Control
 */
PARAM_DEFINE_FLOAT(MC_INDI_RKP_X, 25.0f);

/**
 * INDI pitch rate-to-acceleration gain
 *
 * Maps body-rate error to desired pitch angular acceleration.
 *
 * @unit 1/s
 * @min 0
 * @max 200
 * @decimal 2
 * @group Multicopter INDI Control
 */
PARAM_DEFINE_FLOAT(MC_INDI_RKP_Y, 25.0f);

/**
 * INDI yaw rate-to-acceleration gain
 *
 * Maps body-rate error to desired yaw angular acceleration.
 *
 * @unit 1/s
 * @min 0
 * @max 200
 * @decimal 2
 * @group Multicopter INDI Control
 */
PARAM_DEFINE_FLOAT(MC_INDI_RKP_Z, 15.0f);

/**
 * INDI roll angular acceleration limit
 *
 * @unit rad/s^2
 * @min 1
 * @max 5000
 * @decimal 1
 * @group Multicopter INDI Control
 */
PARAM_DEFINE_FLOAT(MC_INDI_NLIM_X, 400.0f);

/**
 * INDI pitch angular acceleration limit
 *
 * @unit rad/s^2
 * @min 1
 * @max 5000
 * @decimal 1
 * @group Multicopter INDI Control
 */
PARAM_DEFINE_FLOAT(MC_INDI_NLIM_Y, 400.0f);

/**
 * INDI yaw angular acceleration limit
 *
 * @unit rad/s^2
 * @min 1
 * @max 5000
 * @decimal 1
 * @group Multicopter INDI Control
 */
PARAM_DEFINE_FLOAT(MC_INDI_NLIM_Z, 200.0f);

/**
 * INDI roll normalized torque limit
 *
 * @min 0
 * @max 1
 * @decimal 3
 * @group Multicopter INDI Control
 */
PARAM_DEFINE_FLOAT(MC_INDI_TLIM_X, 1.0f);

/**
 * INDI pitch normalized torque limit
 *
 * @min 0
 * @max 1
 * @decimal 3
 * @group Multicopter INDI Control
 */
PARAM_DEFINE_FLOAT(MC_INDI_TLIM_Y, 1.0f);

/**
 * INDI yaw normalized torque limit
 *
 * @min 0
 * @max 1
 * @decimal 3
 * @group Multicopter INDI Control
 */
PARAM_DEFINE_FLOAT(MC_INDI_TLIM_Z, 1.0f);

/**
 * INDI roll normalized torque slew rate
 *
 * Limits the change rate of the final normalized roll torque command. Set to
 * zero to disable slew limiting.
 *
 * @unit 1/s
 * @min 0
 * @max 1000
 * @decimal 1
 * @group Multicopter INDI Control
 */
PARAM_DEFINE_FLOAT(MC_INDI_SLEW_X, 0.0f);

/**
 * INDI pitch normalized torque slew rate
 *
 * Limits the change rate of the final normalized pitch torque command. Set to
 * zero to disable slew limiting.
 *
 * @unit 1/s
 * @min 0
 * @max 1000
 * @decimal 1
 * @group Multicopter INDI Control
 */
PARAM_DEFINE_FLOAT(MC_INDI_SLEW_Y, 0.0f);

/**
 * INDI yaw normalized torque slew rate
 *
 * Limits the change rate of the final normalized yaw torque command. Set to
 * zero to disable slew limiting.
 *
 * @unit 1/s
 * @min 0
 * @max 1000
 * @decimal 1
 * @group Multicopter INDI Control
 */
PARAM_DEFINE_FLOAT(MC_INDI_SLEW_Z, 0.0f);

/**
 * INDI actuator state source
 *
 * Selects the applied-torque estimate source. Output and RPM paths are reserved
 * for follow-up implementation; the command model is currently used as fallback.
 *
 * @value 0 First-order command model
 * @value 1 Actuator outputs, fallback to command model
 * @value 2 ESC RPM, fallback to command model
 * @min 0
 * @max 2
 * @group Multicopter INDI Control
 */
PARAM_DEFINE_INT32(MC_INDI_ACT_SRC, 0);

/**
 * INDI actuator model time constant
 *
 * First-order motor/actuator time constant used for applied torque estimate.
 *
 * @unit s
 * @min 0.001
 * @max 0.5
 * @decimal 3
 * @group Multicopter INDI Control
 */
PARAM_DEFINE_FLOAT(MC_INDI_MOT_TC, 0.04f);

/**
 * INDI compensation enable bitmask
 *
 * Bit 0 enables angular acceleration compensation, bit 1 enables physical
 * torque compensation.
 *
 * @bit 0 angular acceleration compensation
 * @bit 1 torque compensation
 * @min 0
 * @max 3
 * @group Multicopter INDI Control
 */
PARAM_DEFINE_INT32(MC_INDI_COMP_EN, 0);

/**
 * INDI compensation timeout
 *
 * Timeout for the onboard compensation stream.
 *
 * @unit ms
 * @min 5
 * @max 1000
 * @group Multicopter INDI Control
 */
PARAM_DEFINE_INT32(MC_INDI_COMP_TO, 50);

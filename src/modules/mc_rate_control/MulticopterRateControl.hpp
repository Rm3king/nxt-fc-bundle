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

#pragma once

#include "IndiActuatorModel.hpp"

#include <lib/rate_control/rate_control.hpp>
#include <lib/system_identification/signal_generator.hpp>
#include <lib/geo/geo.h>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <lib/mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/systemlib/mavlink_log.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_controls_status.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/control_allocator_status.h>
#include <uORB/topics/indi_setpoint.h>
#include <uORB/topics/indi_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/rate_ctrl_compensation.h>
#include <uORB/topics/rate_sysid_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

using namespace time_literals;

class MulticopterRateControl : public ModuleBase<MulticopterRateControl>, public ModuleParams, public px4::WorkItem
{
public:
	MulticopterRateControl(bool vtol = false);
	~MulticopterRateControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	/**
	 * initialize some vectors/matrices from parameters
	 */
	void parameters_updated();

	void updateActuatorControlsStatus(const vehicle_torque_setpoint_s &vehicle_torque_setpoint, float dt);

	bool updateAccelHeadingFrontend(const matrix::Quatf &q, matrix::Vector3f &rates_setpoint,
					matrix::Vector3f &thrust_setpoint, matrix::Vector3f &angular_accel_ff,
					matrix::Quatf &q_sp, matrix::Vector3f &accel_sp_consumed, float &heading_sp);
	matrix::Vector3f attitudeRatesSetpoint(const matrix::Quatf &q, const matrix::Quatf &qd) const;
	void updateRateControlCompensation(uint8_t &flags, matrix::Vector3f &nu_comp,
					   matrix::Vector3f &tau_comp);
	matrix::Vector3f updateIndiControl(uint64_t timestamp_sample, const matrix::Vector3f &rates,
					   const matrix::Vector3f &rates_setpoint,
					   const matrix::Vector3f &angular_accel_raw, const matrix::Vector3f &angular_accel_ff,
					   float dt, const matrix::Vector3f &unallocated_torque_norm, bool allocator_feedback_valid,
					   uint8_t &flags, matrix::Vector3f &omega_dot_raw_used, matrix::Vector3f &omega_dot_f,
					   matrix::Vector3f &nu, const matrix::Vector3f &nu_comp, matrix::Vector3f &tau0_f,
					   const matrix::Vector3f &tau_comp, matrix::Vector3f &tau_phys,
					   matrix::Vector3f &tau_norm);
	void publishIndiStatus(uint64_t timestamp_sample, uint8_t mode, uint8_t setpoint_source, uint8_t flags,
			       const matrix::Vector3f &omega_sp, const matrix::Vector3f &omega,
			       const matrix::Vector3f &omega_dot_raw, const matrix::Vector3f &omega_dot_f,
			       const matrix::Vector3f &nu, const matrix::Vector3f &nu_comp,
			       const matrix::Vector3f &tau0_f, const matrix::Vector3f &tau_comp,
			       const matrix::Vector3f &tau_phys, const matrix::Vector3f &tau_norm,
			       const matrix::Vector3f &thrust_sp, const matrix::Quatf &q_sp,
			       const matrix::Vector3f &accel_sp, float heading_sp);
	void resetIndiState(const matrix::Vector3f &angular_accel, const matrix::Vector3f &tau0);
	bool solveIndiIncrement(const matrix::Vector3f &accel_error, const matrix::Matrix3f &b2_discrete,
				int32_t axis_mask, matrix::Vector3f &command_increment) const;
	void updateIndiCommandHistory(bool allocator_feedback_valid);
	void updateIndiFallback(const matrix::Vector3f &candidate, float dt, bool active, uint8_t &flags);
	float rateSysidSignal(float elapsed, float duration, float &frequency) const;
	matrix::Vector3f updateRateSysid(hrt_abstime timestamp_sample, const matrix::Vector3f &rates,
					  const matrix::Vector3f &base_rate_sp, bool armed_rotary,
					  bool allocator_feedback_valid);
	void setRateSysidState(uint8_t state, hrt_abstime timestamp_sample);
	void abortRateSysid(uint8_t reason, hrt_abstime timestamp_sample);
	void publishRateSysidStatus(hrt_abstime timestamp_sample, const matrix::Vector3f &base_rate_sp,
				    const matrix::Vector3f &rate_sp, const matrix::Vector3f &rates,
				    const matrix::Vector3f &torque_sp_pid, const matrix::Vector3f &torque_sp);
	float selectedRateSysidAux() const;
	bool rateSysidRunning() const;
	uint8_t rateSysidAxis() const;

	RateControl _rate_control; ///< class for rate control calculations

	uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};
	uORB::Subscription _actuator_motors_sub{ORB_ID(actuator_motors)};
	uORB::Subscription _control_allocator_status_sub{ORB_ID(control_allocator_status)};
	uORB::Subscription _indi_setpoint_sub{ORB_ID(indi_setpoint)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _rate_sysid_manual_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _rate_ctrl_compensation_sub{ORB_ID(rate_ctrl_compensation)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _vehicle_rates_setpoint_sub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};

	uORB::Publication<actuator_controls_status_s>	_actuator_controls_status_pub{ORB_ID(actuator_controls_status_0)};
	uORB::Publication<indi_status_s>	_indi_status_pub{ORB_ID(indi_status)};
	uORB::Publication<rate_sysid_status_s> _rate_sysid_status_pub{ORB_ID(rate_sysid_status)};
	uORB::PublicationMulti<rate_ctrl_status_s>	_controller_status_pub{ORB_ID(rate_ctrl_status)};
	uORB::Publication<vehicle_rates_setpoint_s>	_vehicle_rates_setpoint_pub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Publication<vehicle_torque_setpoint_s>	_vehicle_torque_setpoint_pub;
	uORB::Publication<vehicle_thrust_setpoint_s>	_vehicle_thrust_setpoint_pub;

	vehicle_control_mode_s	_vehicle_control_mode{};
	vehicle_status_s	_vehicle_status{};
	vehicle_attitude_s	_vehicle_attitude{};
	indi_setpoint_s		_indi_setpoint{};
	actuator_motors_s	_actuator_motors{};
	rate_ctrl_compensation_s _rate_ctrl_compensation{};
	hrt_abstime _indi_setpoint_rx_time{0};
	hrt_abstime _rate_ctrl_compensation_rx_time{0};

	bool _landed{true};
	bool _maybe_landed{true};

	hrt_abstime _last_run{0};
	hrt_abstime _indi_status_last_pub{0};

	perf_counter_t	_loop_perf;			/**< loop duration performance counter */

	// keep setpoint values between updates
	matrix::Vector3f _acro_rate_max;		/**< max attitude rates in acro mode */
	matrix::Vector3f _rates_setpoint{};

	float _battery_status_scale{0.0f};
	matrix::Vector3f _thrust_setpoint{};

	matrix::Vector3f _indi_inertia{};
	matrix::Vector3f _indi_torque_max{};
	matrix::Vector3f _indi_attitude_gain{};
	matrix::Vector3f _indi_rate_gain{};
	matrix::Vector3f _indi_angular_accel_limit{};
	matrix::Vector3f _indi_torque_norm_limit{};
	matrix::Vector3f _indi_torque_slew_rate{};
	matrix::Matrix3f _indi_b1{};
	// MC_INDI_B2_* stores the continuous G2 coefficient. It is converted to
	// the discrete B2=G2/dt inside updateIndiControl() at the gyro loop rate.
	matrix::Matrix3f _indi_b2{};
	matrix::Vector3f _allocator_unallocated_torque{};
	hrt_abstime _allocator_status_timestamp{0};

	// INDI synchronized filters: second-order Butterworth (LowPassFilter2p) so the
	// omega_dot / tau0 pair rolls off steeply above MC_INDI_FILT. The previous
	// single-pole AlphaFilter passed too much of the ~40 Hz differentiation noise
	// and let the inner loop self-excite into a limit cycle.
	math::LowPassFilter2p<matrix::Vector3f> _indi_omega_dot_filter{};
	math::LowPassFilter2p<matrix::Vector3f> _indi_tau0_filter{};
	math::LowPassFilter2p<matrix::Vector3f> _indi_actuator_filter{};
	math::LowPassFilter2p<matrix::Vector3f> _indi_hf_low_filter{};
	math::LowPassFilter2p<matrix::Vector3f> _indi_hf_high_filter{};
	// Cache the configured (sample_freq, cutoff) so we only call set_cutoff_frequency()
	// when it actually changes. LowPassFilter2p::set_cutoff_frequency() clears the
	// delay elements on every call, so calling it each loop iteration would wipe the
	// filter state every sample and effectively disable the low-pass.
	float _indi_filter_sample_freq{-1.f};
	float _indi_filter_cutoff{-1.f};
	float _indi_fs_smooth{-1.f};			///< slow-smoothed loop rate used to configure the biquads
							///< (raw 1/dt jitters ~30% on this FC; configuring off the
							///< instantaneous value would re-clear the delay lines nearly
							///< every sample and silently degrade the 2nd-order filter to 1st)
	matrix::Vector3f _indi_last_omega_dot_f{};	///< last omega_dot filter output (for bumpless re-priming)
	matrix::Vector3f _indi_last_tau0_f{};		///< last tau0 filter output (for bumpless re-priming)
	matrix::Vector3f _indi_tau0_model{};
	matrix::Vector3f _indi_tau_phys_applied_prev{};
	matrix::Vector3f _indi_tau_norm_prev{};
	matrix::Vector3f _indi_actuator_state{};
	matrix::Vector3f _indi_actuator_state_f{};
	matrix::Vector3f _indi_command_increment_prev{};
	float _indi_command_increment_dt_prev{0.f};
	matrix::Vector3f _indi_b2_accel{};
	matrix::Vector3f _indi_hf_energy{};
	matrix::Vector3f _indi_published_torque{};
	uint64_t _indi_published_torque_timestamp{0};
	IndiActuatorModel _indi_actuator_model{};
	matrix::Vector3f _indi_rates_prev{};
	matrix::Quatf _indi_last_q_sp{1.f, 0.f, 0.f, 0.f};
	matrix::Vector3f _indi_last_accel_sp{};
	float _indi_last_heading_sp{0.f};
	bool _indi_filters_initialized{false};
	bool _indi_tau_norm_prev_valid{false};
	bool _indi_rates_prev_valid{false};
	bool _indi_published_torque_valid{false};
	bool _indi_active_prev{false};
	bool _indi_dynamic_model_ready{false};
	bool _indi_fallback_latched{false};
	uint8_t _indi_saturation_mask{0};
	uint8_t _indi_fallback_reason{indi_status_s::FALLBACK_NONE};
	hrt_abstime _indi_saturation_since{0};
	hrt_abstime _indi_allocation_since{0};
	hrt_abstime _indi_hf_since{0};
	float _indi_sensor_dt{0.f};

	manual_control_setpoint_s _rate_sysid_manual{};
	matrix::Vector3f _rate_sysid_injection{};
	matrix::Vector3f _rate_sysid_torque_injection{};
	matrix::Vector3f _rate_sysid_pid_torque{};
	matrix::Vector3f _rate_sysid_last_torque{};
	matrix::Vector3f _rate_sysid_initial_body_z{0.f, 0.f, 1.f};
	hrt_abstime _rate_sysid_state_start{0};
	hrt_abstime _rate_sysid_status_last_pub{0};
	hrt_abstime _rate_sysid_allocation_bad_since{0};
	uint8_t _rate_sysid_state{rate_sysid_status_s::STATE_IDLE};
	uint8_t _rate_sysid_abort_reason{rate_sysid_status_s::ABORT_NONE};
	bool _rate_sysid_switch_ready{false};
	bool _rate_sysid_offboard_at_start{false};
	float _rate_sysid_frequency{0.f};

	static constexpr float kRateSysidSettleTime{3.f};
	static constexpr float kRateSysidPauseTime{2.f};
	static constexpr float kRateSysidRampTime{0.75f};
	static constexpr float kRateSysidMaxRate{1.f};
	// Command error is only a final sanity guard. Actual body rate, torque,
	// attitude and allocator limits below remain the primary safety guards.
	static constexpr float kRateSysidMaxRateError{2.f};
	static constexpr float kRateSysidMaxTorque{0.18f};
	static constexpr float kRateSysidMaxUnallocatedTorque{0.05f};
	static constexpr float kRateSysidMaxTiltDelta{0.21f};

	float _energy_integration_time{0.0f};
	float _control_energy[4] {};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MC_ROLLRATE_P>) _param_mc_rollrate_p,
		(ParamFloat<px4::params::MC_ROLLRATE_I>) _param_mc_rollrate_i,
		(ParamFloat<px4::params::MC_RR_INT_LIM>) _param_mc_rr_int_lim,
		(ParamFloat<px4::params::MC_ROLLRATE_D>) _param_mc_rollrate_d,
		(ParamFloat<px4::params::MC_ROLLRATE_FF>) _param_mc_rollrate_ff,
		(ParamFloat<px4::params::MC_ROLLRATE_K>) _param_mc_rollrate_k,

		(ParamFloat<px4::params::MC_PITCHRATE_P>) _param_mc_pitchrate_p,
		(ParamFloat<px4::params::MC_PITCHRATE_I>) _param_mc_pitchrate_i,
		(ParamFloat<px4::params::MC_PR_INT_LIM>) _param_mc_pr_int_lim,
		(ParamFloat<px4::params::MC_PITCHRATE_D>) _param_mc_pitchrate_d,
		(ParamFloat<px4::params::MC_PITCHRATE_FF>) _param_mc_pitchrate_ff,
		(ParamFloat<px4::params::MC_PITCHRATE_K>) _param_mc_pitchrate_k,

		(ParamFloat<px4::params::MC_YAWRATE_P>) _param_mc_yawrate_p,
		(ParamFloat<px4::params::MC_YAWRATE_I>) _param_mc_yawrate_i,
		(ParamFloat<px4::params::MC_YR_INT_LIM>) _param_mc_yr_int_lim,
		(ParamFloat<px4::params::MC_YAWRATE_D>) _param_mc_yawrate_d,
		(ParamFloat<px4::params::MC_YAWRATE_FF>) _param_mc_yawrate_ff,
		(ParamFloat<px4::params::MC_YAWRATE_K>) _param_mc_yawrate_k,

		(ParamFloat<px4::params::MC_ACRO_R_MAX>) _param_mc_acro_r_max,
		(ParamFloat<px4::params::MC_ACRO_P_MAX>) _param_mc_acro_p_max,
		(ParamFloat<px4::params::MC_ACRO_Y_MAX>) _param_mc_acro_y_max,
		(ParamFloat<px4::params::MC_ACRO_EXPO>) _param_mc_acro_expo,			/**< expo stick curve shape (roll & pitch) */
		(ParamFloat<px4::params::MC_ACRO_EXPO_Y>) _param_mc_acro_expo_y,				/**< expo stick curve shape (yaw) */
		(ParamFloat<px4::params::MC_ACRO_SUPEXPO>) _param_mc_acro_supexpo,		/**< superexpo stick curve shape (roll & pitch) */
		(ParamFloat<px4::params::MC_ACRO_SUPEXPOY>) _param_mc_acro_supexpoy,		/**< superexpo stick curve shape (yaw) */

		(ParamBool<px4::params::MC_BAT_SCALE_EN>) _param_mc_bat_scale_en,
		(ParamInt<px4::params::MC_RSYSID_AUX>) _param_mc_rsysid_aux,
		(ParamInt<px4::params::MC_RSYSID_MODE>) _param_mc_rsysid_mode,
		(ParamFloat<px4::params::MC_RSYSID_AMP>) _param_mc_rsysid_amp,
		(ParamFloat<px4::params::MC_RSYSID_TAMP>) _param_mc_rsysid_tamp,
		(ParamFloat<px4::params::MC_RSYSID_F0>) _param_mc_rsysid_f0,
		(ParamFloat<px4::params::MC_RSYSID_F1>) _param_mc_rsysid_f1,
		(ParamFloat<px4::params::MC_RSYSID_T>) _param_mc_rsysid_t,
		(ParamInt<px4::params::MC_RSYSID_REP>) _param_mc_rsysid_rep,

		(ParamInt<px4::params::MC_INDI_MODE>) _param_mc_indi_mode,
		(ParamBool<px4::params::MC_INDI_ONLY_OF>) _param_mc_indi_only_of,
		(ParamInt<px4::params::MC_INDI_SP_MODE>) _param_mc_indi_sp_mode,
		(ParamInt<px4::params::MC_INDI_SP_TO>) _param_mc_indi_sp_to,
		(ParamFloat<px4::params::MC_INDI_MASS>) _param_mc_indi_mass,
		(ParamFloat<px4::params::MC_INDI_HOV_THR>) _param_mc_indi_hov_thr,
		(ParamFloat<px4::params::MC_INDI_FILT>) _param_mc_indi_filt,
		(ParamFloat<px4::params::MC_INDI_JX>) _param_mc_indi_jx,
		(ParamFloat<px4::params::MC_INDI_JY>) _param_mc_indi_jy,
		(ParamFloat<px4::params::MC_INDI_JZ>) _param_mc_indi_jz,
		(ParamFloat<px4::params::MC_INDI_TMAX_X>) _param_mc_indi_tmax_x,
		(ParamFloat<px4::params::MC_INDI_TMAX_Y>) _param_mc_indi_tmax_y,
		(ParamFloat<px4::params::MC_INDI_TMAX_Z>) _param_mc_indi_tmax_z,
		(ParamFloat<px4::params::MC_INDI_AKP_X>) _param_mc_indi_akp_x,
		(ParamFloat<px4::params::MC_INDI_AKP_Y>) _param_mc_indi_akp_y,
		(ParamFloat<px4::params::MC_INDI_AKP_Z>) _param_mc_indi_akp_z,
		(ParamFloat<px4::params::MC_INDI_YAW_W>) _param_mc_indi_yaw_w,
		(ParamFloat<px4::params::MC_INDI_RKP_X>) _param_mc_indi_rkp_x,
		(ParamFloat<px4::params::MC_INDI_RKP_Y>) _param_mc_indi_rkp_y,
		(ParamFloat<px4::params::MC_INDI_RKP_Z>) _param_mc_indi_rkp_z,
		(ParamFloat<px4::params::MC_INDI_NLIM_X>) _param_mc_indi_nlim_x,
		(ParamFloat<px4::params::MC_INDI_NLIM_Y>) _param_mc_indi_nlim_y,
		(ParamFloat<px4::params::MC_INDI_NLIM_Z>) _param_mc_indi_nlim_z,
		(ParamFloat<px4::params::MC_INDI_TLIM_X>) _param_mc_indi_tlim_x,
		(ParamFloat<px4::params::MC_INDI_TLIM_Y>) _param_mc_indi_tlim_y,
		(ParamFloat<px4::params::MC_INDI_TLIM_Z>) _param_mc_indi_tlim_z,
		(ParamFloat<px4::params::MC_INDI_SLEW_X>) _param_mc_indi_slew_x,
		(ParamFloat<px4::params::MC_INDI_SLEW_Y>) _param_mc_indi_slew_y,
		(ParamFloat<px4::params::MC_INDI_SLEW_Z>) _param_mc_indi_slew_z,
		(ParamInt<px4::params::MC_INDI_ACT_SRC>) _param_mc_indi_act_src,
		(ParamInt<px4::params::MC_INDI_AXES>) _param_mc_indi_axes,
		(ParamFloat<px4::params::MC_INDI_MOT_TC>) _param_mc_indi_mot_tc,
		(ParamFloat<px4::params::MC_INDI_MOT_DLY>) _param_mc_indi_mot_dly,
		(ParamFloat<px4::params::MC_INDI_B1_RR>) _param_mc_indi_b1_rr,
		(ParamFloat<px4::params::MC_INDI_B1_RP>) _param_mc_indi_b1_rp,
		(ParamFloat<px4::params::MC_INDI_B1_RY>) _param_mc_indi_b1_ry,
		(ParamFloat<px4::params::MC_INDI_B1_PR>) _param_mc_indi_b1_pr,
		(ParamFloat<px4::params::MC_INDI_B1_PP>) _param_mc_indi_b1_pp,
		(ParamFloat<px4::params::MC_INDI_B1_PY>) _param_mc_indi_b1_py,
		(ParamFloat<px4::params::MC_INDI_B1_YR>) _param_mc_indi_b1_yr,
		(ParamFloat<px4::params::MC_INDI_B1_YP>) _param_mc_indi_b1_yp,
		(ParamFloat<px4::params::MC_INDI_B1_YY>) _param_mc_indi_b1_yy,
		(ParamFloat<px4::params::MC_INDI_B2_YR>) _param_mc_indi_b2_yr,
		(ParamFloat<px4::params::MC_INDI_B2_YP>) _param_mc_indi_b2_yp,
		(ParamFloat<px4::params::MC_INDI_B2_YY>) _param_mc_indi_b2_yy,
		(ParamFloat<px4::params::MC_INDI_HF_LIM>) _param_mc_indi_hf_lim,
		(ParamInt<px4::params::MC_INDI_COMP_EN>) _param_mc_indi_comp_en,
		(ParamInt<px4::params::MC_INDI_COMP_TO>) _param_mc_indi_comp_to
	)
};

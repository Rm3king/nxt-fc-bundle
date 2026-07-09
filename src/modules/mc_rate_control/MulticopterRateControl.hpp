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

#include <lib/rate_control/rate_control.hpp>
#include <lib/geo/geo.h>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
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
#include <uORB/topics/battery_status.h>
#include <uORB/topics/control_allocator_status.h>
#include <uORB/topics/indi_setpoint.h>
#include <uORB/topics/indi_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/rate_ctrl_compensation.h>
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
	matrix::Vector3f updateIndiControl(const matrix::Vector3f &rates, const matrix::Vector3f &rates_setpoint,
					   const matrix::Vector3f &angular_accel_raw, const matrix::Vector3f &angular_accel_ff,
					   float dt, const matrix::Vector3f &unallocated_torque_norm, bool allocator_feedback_valid,
					   uint8_t &flags, matrix::Vector3f &omega_dot_raw_used, matrix::Vector3f &omega_dot_f,
					   matrix::Vector3f &nu, matrix::Vector3f &nu_comp, matrix::Vector3f &tau0_f,
					   matrix::Vector3f &tau_comp, matrix::Vector3f &tau_phys, matrix::Vector3f &tau_norm);
	void publishIndiStatus(uint64_t timestamp_sample, uint8_t mode, uint8_t setpoint_source, uint8_t flags,
			       const matrix::Vector3f &omega_sp, const matrix::Vector3f &omega,
			       const matrix::Vector3f &omega_dot_raw, const matrix::Vector3f &omega_dot_f,
			       const matrix::Vector3f &nu, const matrix::Vector3f &nu_comp,
			       const matrix::Vector3f &tau0_f, const matrix::Vector3f &tau_comp,
			       const matrix::Vector3f &tau_phys, const matrix::Vector3f &tau_norm,
			       const matrix::Vector3f &thrust_sp, const matrix::Quatf &q_sp,
			       const matrix::Vector3f &accel_sp, float heading_sp);
	void resetIndiState(const matrix::Vector3f &angular_accel, const matrix::Vector3f &tau0);

	RateControl _rate_control; ///< class for rate control calculations

	uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};
	uORB::Subscription _control_allocator_status_sub{ORB_ID(control_allocator_status)};
	uORB::Subscription _indi_setpoint_sub{ORB_ID(indi_setpoint)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
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
	uORB::PublicationMulti<rate_ctrl_status_s>	_controller_status_pub{ORB_ID(rate_ctrl_status)};
	uORB::Publication<vehicle_rates_setpoint_s>	_vehicle_rates_setpoint_pub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Publication<vehicle_torque_setpoint_s>	_vehicle_torque_setpoint_pub;
	uORB::Publication<vehicle_thrust_setpoint_s>	_vehicle_thrust_setpoint_pub;

	vehicle_control_mode_s	_vehicle_control_mode{};
	vehicle_status_s	_vehicle_status{};
	vehicle_attitude_s	_vehicle_attitude{};
	indi_setpoint_s		_indi_setpoint{};
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
	matrix::Vector3f _allocator_unallocated_torque{};
	hrt_abstime _allocator_status_timestamp{0};

	AlphaFilter<matrix::Vector3f> _indi_omega_dot_filter{};
	AlphaFilter<matrix::Vector3f> _indi_tau0_filter{};
	matrix::Vector3f _indi_tau0_model{};
	matrix::Vector3f _indi_tau_phys_applied_prev{};
	matrix::Vector3f _indi_tau_norm_prev{};
	matrix::Vector3f _indi_rates_prev{};
	matrix::Quatf _indi_last_q_sp{1.f, 0.f, 0.f, 0.f};
	matrix::Vector3f _indi_last_accel_sp{};
	float _indi_last_heading_sp{0.f};
	bool _indi_filters_initialized{false};
	bool _indi_tau_norm_prev_valid{false};
	bool _indi_rates_prev_valid{false};

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
		(ParamFloat<px4::params::MC_INDI_MOT_TC>) _param_mc_indi_mot_tc,
		(ParamInt<px4::params::MC_INDI_COMP_EN>) _param_mc_indi_comp_en,
		(ParamInt<px4::params::MC_INDI_COMP_TO>) _param_mc_indi_comp_to
	)
};

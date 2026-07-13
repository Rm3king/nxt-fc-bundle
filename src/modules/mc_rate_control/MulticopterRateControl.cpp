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

#include "MulticopterRateControl.hpp"

#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <px4_platform_common/events.h>

using namespace matrix;
using namespace time_literals;
using math::radians;

MulticopterRateControl::MulticopterRateControl(bool vtol) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_vehicle_torque_setpoint_pub(vtol ? ORB_ID(vehicle_torque_setpoint_virtual_mc) : ORB_ID(vehicle_torque_setpoint)),
	_vehicle_thrust_setpoint_pub(vtol ? ORB_ID(vehicle_thrust_setpoint_virtual_mc) : ORB_ID(vehicle_thrust_setpoint)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

	parameters_updated();
	_controller_status_pub.advertise();
}

MulticopterRateControl::~MulticopterRateControl()
{
	perf_free(_loop_perf);
}

bool
MulticopterRateControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void
MulticopterRateControl::parameters_updated()
{
	// rate control parameters
	// The controller gain K is used to convert the parallel (P + I/s + sD) form
	// to the ideal (K * [1 + 1/sTi + sTd]) form
	const Vector3f rate_k = Vector3f(_param_mc_rollrate_k.get(), _param_mc_pitchrate_k.get(), _param_mc_yawrate_k.get());

	_rate_control.setPidGains(
		rate_k.emult(Vector3f(_param_mc_rollrate_p.get(), _param_mc_pitchrate_p.get(), _param_mc_yawrate_p.get())),
		rate_k.emult(Vector3f(_param_mc_rollrate_i.get(), _param_mc_pitchrate_i.get(), _param_mc_yawrate_i.get())),
		rate_k.emult(Vector3f(_param_mc_rollrate_d.get(), _param_mc_pitchrate_d.get(), _param_mc_yawrate_d.get())));

	_rate_control.setIntegratorLimit(
		Vector3f(_param_mc_rr_int_lim.get(), _param_mc_pr_int_lim.get(), _param_mc_yr_int_lim.get()));

	_rate_control.setFeedForwardGain(
		Vector3f(_param_mc_rollrate_ff.get(), _param_mc_pitchrate_ff.get(), _param_mc_yawrate_ff.get()));


	// manual rate control acro mode rate limits
	_acro_rate_max = Vector3f(radians(_param_mc_acro_r_max.get()), radians(_param_mc_acro_p_max.get()),
				  radians(_param_mc_acro_y_max.get()));

	_indi_inertia = Vector3f(_param_mc_indi_jx.get(), _param_mc_indi_jy.get(), _param_mc_indi_jz.get());
	_indi_torque_max = Vector3f(_param_mc_indi_tmax_x.get(), _param_mc_indi_tmax_y.get(), _param_mc_indi_tmax_z.get());
	_indi_attitude_gain = Vector3f(_param_mc_indi_akp_x.get(), _param_mc_indi_akp_y.get(), _param_mc_indi_akp_z.get());
	_indi_rate_gain = Vector3f(_param_mc_indi_rkp_x.get(), _param_mc_indi_rkp_y.get(), _param_mc_indi_rkp_z.get());
	_indi_angular_accel_limit = Vector3f(_param_mc_indi_nlim_x.get(), _param_mc_indi_nlim_y.get(),
					     _param_mc_indi_nlim_z.get());
	_indi_torque_norm_limit = Vector3f(_param_mc_indi_tlim_x.get(), _param_mc_indi_tlim_y.get(),
					   _param_mc_indi_tlim_z.get());
	_indi_torque_slew_rate = Vector3f(_param_mc_indi_slew_x.get(), _param_mc_indi_slew_y.get(),
					  _param_mc_indi_slew_z.get());

	_indi_b1 = Matrix3f{};
	_indi_b1(0, 0) = _param_mc_indi_b1_rr.get();
	_indi_b1(0, 1) = _param_mc_indi_b1_rp.get();
	_indi_b1(0, 2) = _param_mc_indi_b1_ry.get();
	_indi_b1(1, 0) = _param_mc_indi_b1_pr.get();
	_indi_b1(1, 1) = _param_mc_indi_b1_pp.get();
	_indi_b1(1, 2) = _param_mc_indi_b1_py.get();
	_indi_b1(2, 0) = _param_mc_indi_b1_yr.get();
	_indi_b1(2, 1) = _param_mc_indi_b1_yp.get();
	_indi_b1(2, 2) = _param_mc_indi_b1_yy.get();
	_indi_b2 = Matrix3f{};
	_indi_b2(2, 0) = _param_mc_indi_b2_yr.get();
	_indi_b2(2, 1) = _param_mc_indi_b2_yp.get();
	_indi_b2(2, 2) = _param_mc_indi_b2_yy.get();
	const Matrix3f bsum = _indi_b1 + _indi_b2;
	_indi_b_valid = matrix::inv(bsum, _indi_bsum_inv) && _indi_bsum_inv.isAllFinite();
}

void
MulticopterRateControl::resetIndiState(const Vector3f &angular_accel, const Vector3f &tau0)
{
	_indi_tau0_model = tau0;
	_indi_tau_phys_applied_prev = tau0;
	_indi_tau_norm_prev.zero();
	_indi_command_increment_prev.zero();
	_indi_rates_prev.zero();

	// Make sure the biquad coefficients are valid before priming the delay lines.
	// resetIndiState() can be called from the PID branch before updateIndiControl()
	// has had a chance to set the cutoff for the current sample rate.
	if (_indi_omega_dot_filter.get_sample_freq() < 1.f) {
		const float cutoff = math::constrain(_param_mc_indi_filt.get(), 1.f, 400.f);
		_indi_omega_dot_filter.set_cutoff_frequency(1000.f, cutoff);
		_indi_tau0_filter.set_cutoff_frequency(1000.f, cutoff);
		_indi_actuator_filter.set_cutoff_frequency(1000.f, cutoff);
	}

	_indi_omega_dot_filter.reset(angular_accel);
	_indi_tau0_filter.reset(tau0);
	_indi_actuator_filter.reset(_indi_actuator_state);
	_indi_last_omega_dot_f = angular_accel;
	_indi_last_tau0_f = tau0;
	_indi_filters_initialized = true;
	_indi_tau_norm_prev_valid = false;
	_indi_rates_prev_valid = false;
}

bool
MulticopterRateControl::solveIndiIncrement(const Vector3f &accel_error, int32_t axis_mask,
		Vector3f &command_increment) const
{
	command_increment.zero();
	int selected[3] {};
	int count = 0;

	for (int axis = 0; axis < 3; ++axis) {
		if (axis_mask & (1 << axis)) {
			selected[count++] = axis;
		}
	}

	const Matrix3f bsum = _indi_b1 + _indi_b2;

	if (count == 1) {
		const int i = selected[0];
		const float gain = bsum(i, i);

		if (!PX4_ISFINITE(gain) || fabsf(gain) < 1e-4f) {
			return false;
		}

		command_increment(i) = accel_error(i) / gain;

	} else if (count == 2) {
		const int i = selected[0];
		const int j = selected[1];
		const float determinant = bsum(i, i) * bsum(j, j) - bsum(i, j) * bsum(j, i);

		if (!PX4_ISFINITE(determinant) || fabsf(determinant) < 1e-6f) {
			return false;
		}

		command_increment(i) = (bsum(j, j) * accel_error(i) - bsum(i, j) * accel_error(j)) / determinant;
		command_increment(j) = (-bsum(j, i) * accel_error(i) + bsum(i, i) * accel_error(j)) / determinant;

	} else if (count == 3 && _indi_b_valid) {
		command_increment = _indi_bsum_inv * accel_error;

	} else {
		return false;
	}

	return command_increment.isAllFinite();
}

void
MulticopterRateControl::updateIndiCommandHistory(bool allocator_feedback_valid)
{
	if (!_indi_published_torque_valid) {
		return;
	}

	Vector3f achieved = _indi_published_torque;

	if (allocator_feedback_valid) {
		for (int axis = 0; axis < 3; ++axis) {
			const float unallocated = PX4_ISFINITE(_allocator_unallocated_torque(axis)) ?
					  _allocator_unallocated_torque(axis) : 0.f;
			achieved(axis) = math::constrain(achieved(axis) - unallocated, -1.f, 1.f);
		}
	}

	_indi_actuator_model.push(_indi_published_torque_timestamp, achieved);
	_indi_published_torque_valid = false;
}

void
MulticopterRateControl::updateIndiFallback(const Vector3f &candidate, float dt, bool active, uint8_t &flags)
{
	const hrt_abstime now = hrt_absolute_time();
	const float hf_limit = math::constrain(_param_mc_indi_hf_lim.get(), 0.f, 1.f);
	const Vector3f hf_band = _indi_hf_high_filter.apply(candidate) - _indi_hf_low_filter.apply(candidate);
	const float energy_alpha = math::constrain(dt / (0.1f + dt), 0.f, 1.f);
	_indi_hf_energy += energy_alpha * (hf_band.emult(hf_band) - _indi_hf_energy);

	const int32_t axis_mask = _param_mc_indi_axes.get() & 7;
	float selected_hf_rms = 0.f;

	for (int axis = 0; axis < 3; ++axis) {
		if (axis_mask & (1 << axis)) {
			selected_hf_rms = math::max(selected_hf_rms, sqrtf(math::max(_indi_hf_energy(axis), 0.f)));
		}
	}

	const bool saturated = (_indi_saturation_mask & axis_mask) != 0;
	const bool allocator_fresh = _allocator_status_timestamp != 0
				     && hrt_elapsed_time(&_allocator_status_timestamp) < 50_ms;
	bool allocation_bad = false;

	if (allocator_fresh) {
		for (int axis = 0; axis < 3; ++axis) {
			allocation_bad |= (axis_mask & (1 << axis))
					  && fabsf(_allocator_unallocated_torque(axis)) > kRateSysidMaxUnallocatedTorque;
		}
	}
	const bool hf_bad = hf_limit > FLT_EPSILON && selected_hf_rms > hf_limit;

	if (hf_bad) {
		flags |= indi_status_s::FLAG_HF_ENERGY;
	}

	if (!active) {
		_indi_saturation_since = 0;
		_indi_allocation_since = 0;
		_indi_hf_since = 0;
		return;
	}

	auto update_persistence = [now](bool condition, hrt_abstime &since) {
		if (condition) {
			if (since == 0) {
				since = now;
			}

		} else {
			since = 0;
		}
	};

	update_persistence(saturated, _indi_saturation_since);
	update_persistence(allocation_bad, _indi_allocation_since);
	update_persistence(hf_bad, _indi_hf_since);

	if (_indi_fallback_latched) {
		flags |= indi_status_s::FLAG_AUTO_FALLBACK | indi_status_s::FLAG_FALLBACK;
		return;
	}

	if (flags & (indi_status_s::FLAG_MODEL_INVALID | indi_status_s::FLAG_ACTUATOR_STALE)) {
		// Before the delayed command history spans MC_INDI_MOT_DLY, PID keeps
		// flying while the actuator model warms up. A model loss after it has once
		// been valid is a real runtime fault and is latched until reset.
		if (_indi_dynamic_model_ready) {
			_indi_fallback_reason = indi_status_s::FALLBACK_MODEL;
			_indi_fallback_latched = true;
		}

	} else if (_indi_saturation_since != 0 && now - _indi_saturation_since > 100_ms) {
		_indi_fallback_reason = indi_status_s::FALLBACK_SATURATION;
		_indi_fallback_latched = true;

	} else if (_indi_allocation_since != 0 && now - _indi_allocation_since > 50_ms) {
		_indi_fallback_reason = indi_status_s::FALLBACK_ALLOCATION;
		_indi_fallback_latched = true;

	} else if (_indi_hf_since != 0 && now - _indi_hf_since > 100_ms) {
		_indi_fallback_reason = indi_status_s::FALLBACK_HF_ENERGY;
		_indi_fallback_latched = true;
	}

	if (_indi_fallback_latched) {
		flags |= indi_status_s::FLAG_AUTO_FALLBACK | indi_status_s::FLAG_FALLBACK;
	}
}

bool
MulticopterRateControl::rateSysidRunning() const
{
	return _rate_sysid_state >= rate_sysid_status_s::STATE_SETTLE
	       && _rate_sysid_state <= rate_sysid_status_s::STATE_YAW;
}

uint8_t
MulticopterRateControl::rateSysidAxis() const
{
	switch (_rate_sysid_state) {
	case rate_sysid_status_s::STATE_ROLL:
		return rate_sysid_status_s::AXIS_ROLL;

	case rate_sysid_status_s::STATE_PITCH:
		return rate_sysid_status_s::AXIS_PITCH;

	case rate_sysid_status_s::STATE_YAW:
		return rate_sysid_status_s::AXIS_YAW;

	default:
		return rate_sysid_status_s::AXIS_NONE;
	}
}

float
MulticopterRateControl::selectedRateSysidAux() const
{
	switch (_param_mc_rsysid_aux.get()) {
	case 1: return _rate_sysid_manual.aux1;
	case 2: return _rate_sysid_manual.aux2;
	case 3: return _rate_sysid_manual.aux3;
	case 4: return _rate_sysid_manual.aux4;
	case 5: return _rate_sysid_manual.aux5;
	case 6: return _rate_sysid_manual.aux6;
	default: return NAN;
	}
}

void
MulticopterRateControl::setRateSysidState(uint8_t state, hrt_abstime timestamp_sample)
{
	_rate_sysid_state = state;
	_rate_sysid_state_start = timestamp_sample;
	_rate_sysid_injection.zero();
	_rate_sysid_torque_injection.zero();
	_rate_sysid_frequency = 0.f;
}

void
MulticopterRateControl::abortRateSysid(uint8_t reason, hrt_abstime timestamp_sample)
{
	if (rateSysidRunning()) {
		PX4_WARN("rate sysid aborted (%u)", reason);
	}

	_rate_sysid_abort_reason = reason;
	_rate_sysid_allocation_bad_since = 0;
	setRateSysidState(rate_sysid_status_s::STATE_ABORT, timestamp_sample);
}

float
MulticopterRateControl::rateSysidSignal(float elapsed, float duration, float &frequency) const
{
	const float f0 = math::constrain(_param_mc_rsysid_f0.get(), 0.2f, 5.f);
	const float f1 = math::constrain(_param_mc_rsysid_f1.get(), math::max(f0, 1.f), 12.f);

	if (_param_mc_rsysid_mode.get() != rate_sysid_status_s::EXCITATION_TORQUE_MULTISINE) {
		frequency = f0 * powf(f1 / f0, elapsed / math::max(duration, 0.1f));
		return signal_generator::getLogSineSweep(f0, f1, duration, elapsed);
	}

	static constexpr int kComponents{9};
	const int repetitions = math::constrain(_param_mc_rsysid_rep.get(), int32_t{2}, int32_t{8});
	const float period = math::max(duration / repetitions, 1.f);
	const float period_time = fmodf(elapsed, period);
	float sum = 0.f;

	for (int component = 0; component < kComponents; ++component) {
		const float fraction = static_cast<float>(component) / (kComponents - 1);
		const float target_frequency = f0 * powf(f1 / f0, fraction);
		const int bin = math::max(1, static_cast<int>(roundf(target_frequency * period)));
		const float bin_frequency = bin / period;
		const float phase = -M_PI_F * component * (component - 1) / kComponents;
		sum += sinf(2.f * M_PI_F * bin_frequency * period_time + phase);
	}

	frequency = f1;
	return math::constrain(sum / sqrtf(2.f * kComponents), -1.f, 1.f);
}

Vector3f
MulticopterRateControl::updateRateSysid(hrt_abstime timestamp_sample, const Vector3f &rates,
		const Vector3f &base_rate_sp, bool armed_rotary, bool allocator_feedback_valid)
{
	_rate_sysid_manual_sub.update(&_rate_sysid_manual);
	_rate_sysid_injection.zero();
	_rate_sysid_torque_injection.zero();
	_rate_sysid_frequency = 0.f;

	if (_param_mc_rsysid_aux.get() == 0) {
		if (rateSysidRunning()) {
			abortRateSysid(rate_sysid_status_s::ABORT_MODE, timestamp_sample);
		}

		_rate_sysid_switch_ready = false;
		return _rate_sysid_injection;
	}

	const float aux = selectedRateSysidAux();
	const bool rc_fresh = _rate_sysid_manual.valid
			      && _rate_sysid_manual.data_source == manual_control_setpoint_s::SOURCE_RC
			      && _rate_sysid_manual.timestamp != 0
			      && hrt_elapsed_time(&_rate_sysid_manual.timestamp) < 500_ms;
	const bool switch_low = rc_fresh && PX4_ISFINITE(aux) && aux < -0.5f;
	const bool switch_high = rc_fresh && PX4_ISFINITE(aux) && aux > 0.5f;
	const bool attitude_fresh = _vehicle_attitude.timestamp != 0
				    && hrt_elapsed_time(&_vehicle_attitude.timestamp) < 100_ms
				    && PX4_ISFINITE(_vehicle_attitude.q[0])
				    && PX4_ISFINITE(_vehicle_attitude.q[1])
				    && PX4_ISFINITE(_vehicle_attitude.q[2])
				    && PX4_ISFINITE(_vehicle_attitude.q[3]);

	if (switch_low) {
		if (rateSysidRunning()) {
			abortRateSysid(rate_sysid_status_s::ABORT_SWITCH, timestamp_sample);
		}

		_rate_sysid_switch_ready = true;
		return _rate_sysid_injection;
	}

	if (!rateSysidRunning()) {
		if (switch_high && _rate_sysid_switch_ready) {
			_rate_sysid_switch_ready = false;

			if (!rc_fresh) {
				abortRateSysid(rate_sysid_status_s::ABORT_RC_LOSS, timestamp_sample);

			} else if (_param_mc_indi_mode.get() != 0) {
				abortRateSysid(rate_sysid_status_s::ABORT_NOT_PID, timestamp_sample);

			} else if (!armed_rotary || _landed || _maybe_landed) {
				abortRateSysid(rate_sysid_status_s::ABORT_NOT_FLYING, timestamp_sample);

			} else if (!_vehicle_control_mode.flag_control_rates_enabled) {
				abortRateSysid(rate_sysid_status_s::ABORT_MODE, timestamp_sample);

			} else if (!attitude_fresh) {
				abortRateSysid(rate_sysid_status_s::ABORT_ATTITUDE, timestamp_sample);

			} else {
				_rate_sysid_abort_reason = rate_sysid_status_s::ABORT_NONE;
				_rate_sysid_offboard_at_start = _vehicle_control_mode.flag_control_offboard_enabled;
				_rate_sysid_allocation_bad_since = 0;

				_rate_sysid_initial_body_z = Dcmf{Quatf{_vehicle_attitude.q}}.col(2);

				PX4_INFO("rate sysid started");
				setRateSysidState(rate_sysid_status_s::STATE_SETTLE, timestamp_sample);
			}
		}

		return _rate_sysid_injection;
	}

	if (!rc_fresh) {
		abortRateSysid(rate_sysid_status_s::ABORT_RC_LOSS, timestamp_sample);
		return _rate_sysid_injection;
	}

	if (!switch_high) {
		abortRateSysid(rate_sysid_status_s::ABORT_SWITCH, timestamp_sample);
		return _rate_sysid_injection;
	}

	if (_rate_sysid_manual.sticks_moving) {
		abortRateSysid(rate_sysid_status_s::ABORT_STICK_INPUT, timestamp_sample);
		return _rate_sysid_injection;
	}

	if (_param_mc_indi_mode.get() != 0) {
		abortRateSysid(rate_sysid_status_s::ABORT_NOT_PID, timestamp_sample);
		return _rate_sysid_injection;
	}

	if (!armed_rotary || _landed || _maybe_landed) {
		abortRateSysid(rate_sysid_status_s::ABORT_NOT_FLYING, timestamp_sample);
		return _rate_sysid_injection;
	}

	if (!_vehicle_control_mode.flag_control_rates_enabled
	    || _vehicle_control_mode.flag_control_offboard_enabled != _rate_sysid_offboard_at_start) {
		abortRateSysid(rate_sysid_status_s::ABORT_MODE, timestamp_sample);
		return _rate_sysid_injection;
	}

	if (!attitude_fresh) {
		abortRateSysid(rate_sysid_status_s::ABORT_ATTITUDE, timestamp_sample);
		return _rate_sysid_injection;
	}

	if (rates.abs().max() > kRateSysidMaxRate) {
		abortRateSysid(rate_sysid_status_s::ABORT_RATE, timestamp_sample);
		return _rate_sysid_injection;
	}

	if (_rate_sysid_last_torque.abs().max() > kRateSysidMaxTorque) {
		abortRateSysid(rate_sysid_status_s::ABORT_TORQUE, timestamp_sample);
		return _rate_sysid_injection;
	}

	const Vector3f body_z = Dcmf{Quatf{_vehicle_attitude.q}}.col(2);
	const float tilt_delta = acosf(math::constrain(body_z.dot(_rate_sysid_initial_body_z), -1.f, 1.f));

	if (tilt_delta > kRateSysidMaxTiltDelta) {
		abortRateSysid(rate_sysid_status_s::ABORT_ATTITUDE, timestamp_sample);
		return _rate_sysid_injection;
	}

	if (allocator_feedback_valid && _allocator_unallocated_torque.abs().max() > kRateSysidMaxUnallocatedTorque) {
		if (_rate_sysid_allocation_bad_since == 0) {
			_rate_sysid_allocation_bad_since = timestamp_sample;

		} else if (timestamp_sample - _rate_sysid_allocation_bad_since > 50_ms) {
			abortRateSysid(rate_sysid_status_s::ABORT_ALLOCATION, timestamp_sample);
			return _rate_sysid_injection;
		}

	} else {
		_rate_sysid_allocation_bad_since = 0;
	}

	const float duration = math::constrain(_param_mc_rsysid_t.get(), 5.f, 30.f);
	float elapsed = (timestamp_sample - _rate_sysid_state_start) * 1e-6f;

	switch (_rate_sysid_state) {
	case rate_sysid_status_s::STATE_SETTLE:
		if (elapsed >= kRateSysidSettleTime) {
			setRateSysidState(rate_sysid_status_s::STATE_ROLL, timestamp_sample);
			elapsed = 0.f;
		}

		break;

	case rate_sysid_status_s::STATE_ROLL:
		if (elapsed >= duration) {
			setRateSysidState(rate_sysid_status_s::STATE_ROLL_PAUSE, timestamp_sample);
			elapsed = 0.f;
		}

		break;

	case rate_sysid_status_s::STATE_ROLL_PAUSE:
		if (elapsed >= kRateSysidPauseTime) {
			setRateSysidState(rate_sysid_status_s::STATE_PITCH, timestamp_sample);
			elapsed = 0.f;
		}

		break;

	case rate_sysid_status_s::STATE_PITCH:
		if (elapsed >= duration) {
			setRateSysidState(rate_sysid_status_s::STATE_PITCH_PAUSE, timestamp_sample);
			elapsed = 0.f;
		}

		break;

	case rate_sysid_status_s::STATE_PITCH_PAUSE:
		if (elapsed >= kRateSysidPauseTime) {
			setRateSysidState(rate_sysid_status_s::STATE_YAW, timestamp_sample);
			elapsed = 0.f;
		}

		break;

	case rate_sysid_status_s::STATE_YAW:
		if (elapsed >= duration) {
			PX4_INFO("rate sysid complete");
			setRateSysidState(rate_sysid_status_s::STATE_COMPLETE, timestamp_sample);
			return _rate_sysid_injection;
		}

		break;

	default:
		return _rate_sysid_injection;
	}

	const uint8_t axis = rateSysidAxis();

	if (axis != rate_sysid_status_s::AXIS_NONE) {
		const float ramp_time = math::min(kRateSysidRampTime, 0.2f * duration);
		const float ramp_in = math::constrain(elapsed / math::max(ramp_time, 0.1f), 0.f, 1.f);
		const float ramp_out = math::constrain((duration - elapsed) / math::max(ramp_time, 0.1f), 0.f, 1.f);
		const float envelope = 0.5f - 0.5f * cosf(M_PI_F * math::min(ramp_in, ramp_out));
		const float signal = rateSysidSignal(elapsed, duration, _rate_sysid_frequency);
		const int axis_index = static_cast<int>(axis) - 1;

		if (_param_mc_rsysid_mode.get() == rate_sysid_status_s::EXCITATION_RATE_CHIRP) {
			const float amplitude = math::constrain(_param_mc_rsysid_amp.get(), 0.01f, 0.3f);
			_rate_sysid_injection(axis_index) = amplitude * envelope * signal;

		} else {
			const float amplitude = math::constrain(_param_mc_rsysid_tamp.get(), 0.001f, 0.1f);
			_rate_sysid_torque_injection(axis_index) = amplitude * envelope * signal;
		}
	}

	if ((base_rate_sp + _rate_sysid_injection - rates).abs().max() > kRateSysidMaxRateError) {
		abortRateSysid(rate_sysid_status_s::ABORT_RATE_ERROR, timestamp_sample);
	}

	return _rate_sysid_injection;
}

void
MulticopterRateControl::publishRateSysidStatus(hrt_abstime timestamp_sample, const Vector3f &base_rate_sp,
		const Vector3f &rate_sp, const Vector3f &rates, const Vector3f &torque_sp_pid,
		const Vector3f &torque_sp)
{
	const hrt_abstime now = hrt_absolute_time();

	if (_rate_sysid_status_last_pub != 0 && now - _rate_sysid_status_last_pub < 4_ms) {
		return;
	}

	_rate_sysid_status_last_pub = now;
	rate_sysid_status_s status{};
	status.timestamp = now;
	status.timestamp_sample = timestamp_sample;
	status.state = _rate_sysid_state;
	status.axis = rateSysidAxis();
	status.abort_reason = _rate_sysid_abort_reason;
	status.active = rateSysidRunning();
	status.excitation_mode = math::constrain(_param_mc_rsysid_mode.get(), int32_t{0}, int32_t{2});
	status.elapsed_s = _rate_sysid_state_start == 0 ? 0.f : (timestamp_sample - _rate_sysid_state_start) * 1e-6f;
	status.frequency_hz = _rate_sysid_frequency;

	if (status.axis != rate_sysid_status_s::AXIS_NONE) {
		const int axis = static_cast<int>(status.axis) - 1;
		status.injection = status.excitation_mode == rate_sysid_status_s::EXCITATION_RATE_CHIRP ?
				   _rate_sysid_injection(axis) : _rate_sysid_torque_injection(axis);
	}

	_rate_sysid_injection.copyTo(status.rate_injection);
	_rate_sysid_torque_injection.copyTo(status.torque_injection);
	base_rate_sp.copyTo(status.base_rate_sp);
	rate_sp.copyTo(status.rate_sp);
	rates.copyTo(status.measured_rate);
	torque_sp_pid.copyTo(status.torque_sp_pid);
	torque_sp.copyTo(status.torque_sp);
	_rate_sysid_status_pub.publish(status);
}

Vector3f
MulticopterRateControl::attitudeRatesSetpoint(const Quatf &q, const Quatf &qd_in) const
{
	Quatf qd = qd_in;

	const Vector3f e_z = q.dcm_z();
	const Vector3f e_z_d = qd.dcm_z();
	Quatf qd_red(e_z, e_z_d);

	if (fabsf(qd_red(1)) > (1.f - 1e-5f) || fabsf(qd_red(2)) > (1.f - 1e-5f)) {
		qd_red = qd;

	} else {
		qd_red *= q;
	}

	Quatf q_mix = qd_red.inversed() * qd;
	q_mix.canonicalize();
	q_mix(0) = math::constrain(q_mix(0), -1.f, 1.f);
	q_mix(3) = math::constrain(q_mix(3), -1.f, 1.f);

	const float yaw_weight = math::constrain(_param_mc_indi_yaw_w.get(), 0.f, 1.f);
	qd = qd_red * Quatf(cosf(yaw_weight * acosf(q_mix(0))), 0.f, 0.f, sinf(yaw_weight * asinf(q_mix(3))));

	const Quatf qe = q.inversed() * qd;
	Vector3f eq = 2.f * qe.canonical().imag();
	Vector3f rate_setpoint = eq.emult(_indi_attitude_gain);

	for (int i = 0; i < 3; i++) {
		rate_setpoint(i) = math::constrain(rate_setpoint(i), -_acro_rate_max(i), _acro_rate_max(i));
	}

	return rate_setpoint;
}

bool
MulticopterRateControl::updateAccelHeadingFrontend(const Quatf &q, Vector3f &rates_setpoint,
		Vector3f &thrust_setpoint, Vector3f &angular_accel_ff, Quatf &q_sp, Vector3f &accel_sp_consumed,
		float &heading_sp)
{
	if (_indi_setpoint_sub.updated()) {
		_indi_setpoint_sub.copy(&_indi_setpoint);
		_indi_setpoint_rx_time = hrt_absolute_time();
	}

	const hrt_abstime now = hrt_absolute_time();
	const uint64_t setpoint_timeout_us = (uint64_t)math::max(_param_mc_indi_sp_to.get(), (int32_t)10) * 1000ULL;

	if (!_indi_setpoint.valid || _indi_setpoint_rx_time == 0 || now - _indi_setpoint_rx_time > setpoint_timeout_us) {
		return false;
	}

	Vector3f accel_sp{_indi_setpoint.accel_sp};
	const Dcmf R{q};

	if (_indi_setpoint.accel_frame == indi_setpoint_s::ACCEL_FRAME_BODY_FRD) {
		accel_sp = R * accel_sp;

	} else if (_indi_setpoint.accel_frame != indi_setpoint_s::ACCEL_FRAME_NED) {
		return false;
	}

	if (!PX4_ISFINITE(accel_sp(0)) || !PX4_ISFINITE(accel_sp(1)) || !PX4_ISFINITE(accel_sp(2))
	    || !PX4_ISFINITE(_indi_setpoint.heading_sp)) {
		return false;
	}

	const Vector3f thrust_specific_force = accel_sp - Vector3f(0.f, 0.f, CONSTANTS_ONE_G);

	if (thrust_specific_force.norm_squared() < 1e-6f) {
		return false;
	}

	Vector3f body_z = -thrust_specific_force;
	body_z.normalize();

	const float yaw_sp = _indi_setpoint.heading_sp;
	const Vector3f y_C{-sinf(yaw_sp), cosf(yaw_sp), 0.f};
	Vector3f body_x = y_C % body_z;

	if (body_z(2) < 0.f) {
		body_x = -body_x;
	}

	if (fabsf(body_z(2)) < 0.000001f) {
		body_x.zero();
		body_x(2) = 1.f;
	}

	body_x.normalize();
	const Vector3f body_y = body_z % body_x;

	Dcmf R_sp;

	for (int i = 0; i < 3; i++) {
		R_sp(i, 0) = body_x(i);
		R_sp(i, 1) = body_y(i);
		R_sp(i, 2) = body_z(i);
	}

	q_sp = Quatf{R_sp};
	q_sp.normalize();

	const float thrust_min = PX4_ISFINITE(_indi_setpoint.thrust_min) ? math::constrain(_indi_setpoint.thrust_min, 0.f, 1.f) : 0.f;
	const float thrust_max = PX4_ISFINITE(_indi_setpoint.thrust_max) ? math::constrain(_indi_setpoint.thrust_max, thrust_min, 1.f) : 1.f;
	const float hover_thrust = math::constrain(_param_mc_indi_hov_thr.get(), 0.05f, 0.95f);
	const float thrust_magnitude = math::constrain(hover_thrust * thrust_specific_force.norm() / CONSTANTS_ONE_G,
				       thrust_min, thrust_max);

	thrust_setpoint = Vector3f(0.f, 0.f, -thrust_magnitude);

	const Vector3f body_rate_ff{_indi_setpoint.body_rate_ff};
	rates_setpoint = attitudeRatesSetpoint(q, q_sp);

	for (int i = 0; i < 3; i++) {
		if (PX4_ISFINITE(body_rate_ff(i))) {
			rates_setpoint(i) += body_rate_ff(i);
		}

		rates_setpoint(i) = math::constrain(rates_setpoint(i), -_acro_rate_max(i), _acro_rate_max(i));
	}

	angular_accel_ff = Vector3f(_indi_setpoint.body_ang_accel_ff);

	for (int i = 0; i < 3; i++) {
		if (!PX4_ISFINITE(angular_accel_ff(i))) {
			angular_accel_ff(i) = 0.f;
		}
	}

	accel_sp_consumed = accel_sp;
	heading_sp = yaw_sp;

	return true;
}

Vector3f
MulticopterRateControl::updateIndiControl(uint64_t timestamp_sample, const Vector3f &rates,
		const Vector3f &rates_setpoint,
		const Vector3f &angular_accel_raw, const Vector3f &angular_accel_ff, float dt,
		const Vector3f &unallocated_torque_norm, bool allocator_feedback_valid, uint8_t &flags,
		Vector3f &omega_dot_raw_used, Vector3f &omega_dot_f, Vector3f &nu, Vector3f &nu_comp,
		Vector3f &tau0_f, Vector3f &tau_comp, Vector3f &tau_phys, Vector3f &tau_norm)
{
	_indi_saturation_mask = 0;
	_indi_sensor_dt = dt;
	const float sample_freq_raw = 1.f / math::max(dt, 0.000125f);

	// The gyro timestamps on this FC jitter heavily (measured dt std ~30% of the
	// mean), so 1/dt swings sample-to-sample by tens of percent. Configuring the
	// biquads off that raw value would trip the "fs changed" test almost every
	// iteration, and set_cutoff_frequency() clears the delay lines on every call —
	// silently degrading the intended 2nd-order Butterworth back to ~1st order
	// exactly during transients. Configure the filters off a slowly-smoothed loop
	// rate instead; the biquad coefficients are insensitive to sub-ms dt jitter.
	if (_indi_fs_smooth < 0.f) {
		_indi_fs_smooth = sample_freq_raw;

	} else {
		_indi_fs_smooth += 0.01f * (sample_freq_raw - _indi_fs_smooth);
	}

	const float sample_freq = _indi_fs_smooth;
	const float cutoff = math::constrain(_param_mc_indi_filt.get(), 1.f, sample_freq * 0.45f);

	// Only reconfigure the biquads when the (smoothed) sample rate or cutoff
	// meaningfully changes. This now fires only on genuine loop-rate drift or a
	// runtime MC_INDI_FILT change, not on per-sample timestamp jitter.
	const bool cutoff_changed = fabsf(cutoff - _indi_filter_cutoff) > 0.01f;
	const bool fs_changed = _indi_filter_sample_freq < 0.f
				|| fabsf(sample_freq - _indi_filter_sample_freq) > 0.05f * _indi_filter_sample_freq;

	if (cutoff_changed || fs_changed) {
		// Preserve the current filter outputs so the coefficient change is bumpless.
		const Vector3f omega_dot_state = _indi_filters_initialized ? _indi_last_omega_dot_f : angular_accel_raw;
		const Vector3f tau0_state = _indi_filters_initialized ? _indi_last_tau0_f : _indi_tau0_model;

		_indi_omega_dot_filter.set_cutoff_frequency(sample_freq, cutoff);
		_indi_tau0_filter.set_cutoff_frequency(sample_freq, cutoff);
		_indi_actuator_filter.set_cutoff_frequency(sample_freq, cutoff);
		_indi_hf_low_filter.set_cutoff_frequency(sample_freq, math::min(25.f, sample_freq * 0.2f));
		_indi_hf_high_filter.set_cutoff_frequency(sample_freq, math::min(50.f, sample_freq * 0.4f));
		_indi_filter_sample_freq = sample_freq;
		_indi_filter_cutoff = cutoff;

		if (_indi_filters_initialized) {
			// set_cutoff_frequency() cleared the delay lines; re-prime from last output.
			_indi_omega_dot_filter.reset(omega_dot_state);
			_indi_tau0_filter.reset(tau0_state);
			_indi_actuator_filter.reset(_indi_actuator_state_f);
			_indi_hf_low_filter.reset(_indi_tau_norm_prev);
			_indi_hf_high_filter.reset(_indi_tau_norm_prev);
		}
	}

	if (_indi_tau_norm_prev_valid) {
		Vector3f applied_tau_norm = _indi_tau_norm_prev;

		if (allocator_feedback_valid) {
			for (int i = 0; i < 3; i++) {
				const float unallocated = PX4_ISFINITE(unallocated_torque_norm(i)) ? unallocated_torque_norm(i) : 0.f;

				applied_tau_norm(i) -= unallocated;
				const float constrained = math::constrain(applied_tau_norm(i), -1.f, 1.f);

				if (fabsf(constrained - applied_tau_norm(i)) > 1e-5f) {
					flags |= indi_status_s::FLAG_SATURATED;
					_indi_saturation_mask |= 1u << i;
				}

				applied_tau_norm(i) = constrained;
			}
		}

		_indi_tau_phys_applied_prev = applied_tau_norm.emult(_indi_torque_max);
	}

	const int32_t actuator_model = _param_mc_indi_act_src.get();
	const float motor_tau = math::constrain(_param_mc_indi_mot_tc.get(), 0.001f, 0.5f);
	bool dynamic_model_valid = false;

	if (actuator_model == 1) {
		const float motor_delay = math::constrain(_param_mc_indi_mot_dly.get(), 0.f, 50.f) * 1e-3f;
		dynamic_model_valid = _indi_actuator_model.update(timestamp_sample, dt, motor_delay, motor_tau,
				      _indi_actuator_state);
		const uint64_t newest_command = _indi_actuator_model.newestTimestamp();

		if (newest_command == 0 || timestamp_sample < newest_command || timestamp_sample - newest_command > 100_ms) {
			dynamic_model_valid = false;
			flags |= indi_status_s::FLAG_ACTUATOR_STALE;
		}

		_indi_tau0_model = _indi_actuator_state.emult(_indi_torque_max);

	} else {
		const float motor_alpha = math::constrain(dt / (motor_tau + dt), 0.f, 1.f);
		_indi_tau0_model += motor_alpha * (_indi_tau_phys_applied_prev - _indi_tau0_model);
	}

	if (!_indi_filters_initialized) {
		resetIndiState(angular_accel_raw, _indi_tau0_model);
	}

	// INDI requires the measured angular acceleration and previous actuator
	// estimate to be synchronized by the same filter. PX4's
	// vehicle_angular_velocity.xyz_derivative is already filtered upstream by
	// the sensor module, so using it directly here would give omega_dot and tau0
	// different total group delays. Instead, differentiate the exact body-rate
	// sample consumed by this controller and low-pass it in the same frame with
	// the same MC_INDI_FILT coefficient as tau0_model.
	omega_dot_raw_used = angular_accel_raw;

	if (_indi_rates_prev_valid) {
		omega_dot_raw_used = (rates - _indi_rates_prev) / math::max(dt, 0.000125f);
	}

	_indi_rates_prev = rates;
	_indi_rates_prev_valid = true;

	omega_dot_f = _indi_omega_dot_filter.apply(omega_dot_raw_used);

	if (actuator_model == 1) {
		_indi_actuator_state_f = _indi_actuator_filter.apply(_indi_actuator_state);
		tau0_f = _indi_actuator_state_f.emult(_indi_torque_max);

	} else {
		tau0_f = _indi_tau0_filter.apply(_indi_tau0_model);
		_indi_actuator_state = _indi_tau0_model.edivide(_indi_torque_max);
		_indi_actuator_state_f = tau0_f.edivide(_indi_torque_max);
	}
	_indi_last_omega_dot_f = omega_dot_f;
	_indi_last_tau0_f = tau0_f;

	nu = _indi_rate_gain.emult(rates_setpoint - rates) + angular_accel_ff;

	for (int i = 0; i < 3; i++) {
		nu(i) = math::constrain(nu(i), -_indi_angular_accel_limit(i), _indi_angular_accel_limit(i));
	}

	nu_comp.zero();
	tau_comp.zero();

	if (_rate_ctrl_compensation_sub.updated()) {
		_rate_ctrl_compensation_sub.copy(&_rate_ctrl_compensation);
		_rate_ctrl_compensation_rx_time = hrt_absolute_time();
	}

	const hrt_abstime now = hrt_absolute_time();
	const uint64_t comp_timeout_us = (uint64_t)math::max(_param_mc_indi_comp_to.get(), (int32_t)5) * 1000ULL;
	const bool comp_fresh = _rate_ctrl_compensation.valid && _rate_ctrl_compensation_rx_time != 0
				&& now - _rate_ctrl_compensation_rx_time <= comp_timeout_us;

	if (comp_fresh) {
		const int32_t comp_enable = _param_mc_indi_comp_en.get();

		if (comp_enable & 1) {
			nu_comp = Vector3f(_rate_ctrl_compensation.angular_accel_comp);
		}

		if (comp_enable & 2) {
			tau_comp = Vector3f(_rate_ctrl_compensation.torque_comp);
		}

	} else if (_param_mc_indi_comp_en.get() != 0) {
		flags |= indi_status_s::FLAG_COMP_TIMEOUT;
	}

	for (int i = 0; i < 3; i++) {
		if (!PX4_ISFINITE(nu_comp(i))) {
			nu_comp(i) = 0.f;
		}

		if (!PX4_ISFINITE(tau_comp(i))) {
			tau_comp(i) = 0.f;
		}

		const float nu_comp_limited = math::constrain(nu_comp(i), -_indi_angular_accel_limit(i),
					    _indi_angular_accel_limit(i));

		if (fabsf(nu_comp_limited - nu_comp(i)) > 1e-5f) {
			flags |= indi_status_s::FLAG_SATURATED;
			_indi_saturation_mask |= 1u << i;
		}

		nu_comp(i) = nu_comp_limited;

		// The compensation stream is already limited on the companion side.
		// Keep PX4's own pre-limit deliberately high so it does not mask
		// direction tests; the final normalized actuator limit below still
		// protects the actual torque setpoint.
		const float tau_comp_limit = 10.0f;
		const float tau_comp_limited = math::constrain(tau_comp(i), -tau_comp_limit, tau_comp_limit);

		if (fabsf(tau_comp_limited - tau_comp(i)) > 1e-5f) {
			flags |= indi_status_s::FLAG_SATURATED;
			_indi_saturation_mask |= 1u << i;
		}

		tau_comp(i) = tau_comp_limited;
	}

	Vector3f command_increment{};
	const int32_t axis_mask = _param_mc_indi_axes.get() & 7;
	const bool inverse_valid = actuator_model == 1
				   && solveIndiIncrement(nu + nu_comp - omega_dot_f
						   + _indi_b2 * _indi_command_increment_prev,
						   axis_mask, command_increment);

	if (actuator_model == 1 && dynamic_model_valid && inverse_valid) {
		_indi_b2_accel = _indi_b2 * _indi_command_increment_prev;
		tau_norm = _indi_actuator_state_f + command_increment;
		_indi_dynamic_model_ready = true;

		for (int axis = 0; axis < 3; ++axis) {
			tau_norm(axis) += tau_comp(axis) / math::max(_indi_torque_max(axis), 0.0001f);
		}

		tau_phys = tau_norm.emult(_indi_torque_max);

	} else if (actuator_model == 1) {
		_indi_b2_accel.zero();

		if (!inverse_valid) {
			flags |= indi_status_s::FLAG_MODEL_INVALID;
		}

		tau_norm = _indi_tau_norm_prev_valid ? _indi_tau_norm_prev : Vector3f{};
		tau_phys = tau_norm.emult(_indi_torque_max);

	} else {
		_indi_b2_accel.zero();
		tau_phys = tau0_f + _indi_inertia.emult(nu + nu_comp - omega_dot_f) + tau_comp;
	}

	for (int i = 0; i < 3; i++) {
		const float physical_limit = math::max(_indi_torque_max(i) * _indi_torque_norm_limit(i), 0.0001f);
		const float tau_phys_limited = math::constrain(tau_phys(i), -physical_limit, physical_limit);

		if (fabsf(tau_phys_limited - tau_phys(i)) > 1e-5f) {
			flags |= indi_status_s::FLAG_SATURATED;
			_indi_saturation_mask |= 1u << i;
		}

		tau_phys(i) = tau_phys_limited;
		tau_norm(i) = tau_phys(i) / math::max(_indi_torque_max(i), 0.0001f);

		const float constrained = math::constrain(tau_norm(i), -_indi_torque_norm_limit(i), _indi_torque_norm_limit(i));

		if (fabsf(constrained - tau_norm(i)) > 1e-5f) {
			flags |= indi_status_s::FLAG_SATURATED;
			_indi_saturation_mask |= 1u << i;
		}

		tau_norm(i) = constrained;
	}

	if (_indi_tau_norm_prev_valid) {
		for (int i = 0; i < 3; i++) {
			const float slew_rate = _indi_torque_slew_rate(i);

			if (PX4_ISFINITE(slew_rate) && slew_rate > FLT_EPSILON) {
				const float max_delta = slew_rate * dt;
				const float limited = math::constrain(tau_norm(i),
						      _indi_tau_norm_prev(i) - max_delta,
						      _indi_tau_norm_prev(i) + max_delta);

				if (fabsf(limited - tau_norm(i)) > 1e-5f) {
					flags |= indi_status_s::FLAG_SATURATED;
					_indi_saturation_mask |= 1u << i;
				}

				tau_norm(i) = limited;
				tau_phys(i) = tau_norm(i) * math::max(_indi_torque_max(i), 0.0001f);
			}
		}
	}

	_indi_tau_norm_prev = tau_norm;
	_indi_tau_norm_prev_valid = true;

	if (actuator_model == 1) {
		_indi_command_increment_prev = tau_norm - _indi_actuator_state_f;
	}

	return tau_norm;
}

void
MulticopterRateControl::publishIndiStatus(uint64_t timestamp_sample, uint8_t mode, uint8_t setpoint_source,
		uint8_t flags, const Vector3f &omega_sp, const Vector3f &omega, const Vector3f &omega_dot_raw,
		const Vector3f &omega_dot_f, const Vector3f &nu, const Vector3f &nu_comp, const Vector3f &tau0_f,
		const Vector3f &tau_comp, const Vector3f &tau_phys, const Vector3f &tau_norm,
		const Vector3f &thrust_sp, const Quatf &q_sp, const Vector3f &accel_sp, float heading_sp)
{
	const hrt_abstime now = hrt_absolute_time();

	if (_indi_status_last_pub != 0 && now - _indi_status_last_pub < 4_ms) {
		return;
	}

	_indi_status_last_pub = now;

	indi_status_s status{};
	status.timestamp = now;
	status.timestamp_sample = timestamp_sample;
	status.mode = mode;
	status.setpoint_source = setpoint_source;
	status.flags = flags;
	status.fallback_reason = _indi_fallback_reason;
	status.axis_mask = _param_mc_indi_axes.get() & 7;
	status.actuator_model = math::constrain(_param_mc_indi_act_src.get(), int32_t{0}, int32_t{1});
	omega_sp.copyTo(status.omega_sp);
	omega.copyTo(status.omega);
	omega_dot_raw.copyTo(status.omega_dot_raw);
	omega_dot_f.copyTo(status.omega_dot_f);
	nu.copyTo(status.nu);
	nu_comp.copyTo(status.nu_comp);
	_indi_tau0_model.copyTo(status.tau0_model);
	tau0_f.copyTo(status.tau0_f);
	tau_comp.copyTo(status.tau_comp);
	tau_phys.copyTo(status.tau_phys);
	tau_norm.copyTo(status.tau_norm);
	_indi_actuator_state.copyTo(status.actuator_state);
	_indi_actuator_state_f.copyTo(status.actuator_state_f);
	_indi_command_increment_prev.copyTo(status.command_increment);
	_indi_b2_accel.copyTo(status.b2_accel);

	for (int motor = 0; motor < 4; ++motor) {
		status.motor_command[motor] = _actuator_motors.control[motor];
	}

	status.sensor_dt = _indi_sensor_dt;
	status.filter_sample_rate = _indi_filter_sample_freq;
	thrust_sp.copyTo(status.thrust_sp);
	q_sp.copyTo(status.q_sp);
	accel_sp.copyTo(status.accel_sp);
	status.heading_sp = heading_sp;
	_indi_status_pub.publish(status);
}

void
MulticopterRateControl::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
		parameters_updated();
	}

	/* run controller on gyro changes */
	vehicle_angular_velocity_s angular_velocity;

	if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {

		const hrt_abstime now = angular_velocity.timestamp_sample;

		// Guard against too small (< 0.125ms) and too large (> 20ms) dt's.
		const float dt = math::constrain(((now - _last_run) * 1e-6f), 0.000125f, 0.02f);
		_last_run = now;

		const Vector3f rates{angular_velocity.xyz};
		const Vector3f angular_accel{angular_velocity.xyz_derivative};

		/* check for updates in other topics */
		_vehicle_control_mode_sub.update(&_vehicle_control_mode);

		if (_vehicle_land_detected_sub.updated()) {
			vehicle_land_detected_s vehicle_land_detected;

			if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
				_landed = vehicle_land_detected.landed;
				_maybe_landed = vehicle_land_detected.maybe_landed;
			}
		}

		_vehicle_status_sub.update(&_vehicle_status);
		_vehicle_attitude_sub.update(&_vehicle_attitude);

		// use rates setpoint topic
		vehicle_rates_setpoint_s vehicle_rates_setpoint{};
		Vector3f angular_accel_ff{};
		Quatf indi_q_sp = _indi_last_q_sp;
		Vector3f indi_accel_sp = _indi_last_accel_sp;
		float indi_heading_sp = _indi_last_heading_sp;
		uint8_t indi_setpoint_source = indi_status_s::SOURCE_NONE;
		uint8_t indi_flags = 0;

		if (_vehicle_control_mode.flag_control_manual_enabled && !_vehicle_control_mode.flag_control_attitude_enabled) {
			// generate the rate setpoint from sticks
			manual_control_setpoint_s manual_control_setpoint;

			if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
				// manual rates control - ACRO mode
				const Vector3f man_rate_sp{
					math::superexpo(manual_control_setpoint.roll, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
					math::superexpo(-manual_control_setpoint.pitch, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
					math::superexpo(manual_control_setpoint.yaw, _param_mc_acro_expo_y.get(), _param_mc_acro_supexpoy.get())};

				_rates_setpoint = man_rate_sp.emult(_acro_rate_max);
				_thrust_setpoint(2) = -(manual_control_setpoint.throttle + 1.f) * .5f;
				_thrust_setpoint(0) = _thrust_setpoint(1) = 0.f;

				// publish rate setpoint
				vehicle_rates_setpoint.roll = _rates_setpoint(0);
				vehicle_rates_setpoint.pitch = _rates_setpoint(1);
				vehicle_rates_setpoint.yaw = _rates_setpoint(2);
				_thrust_setpoint.copyTo(vehicle_rates_setpoint.thrust_body);
				vehicle_rates_setpoint.timestamp = hrt_absolute_time();

				_vehicle_rates_setpoint_pub.publish(vehicle_rates_setpoint);
			}

		} else if (_vehicle_rates_setpoint_sub.update(&vehicle_rates_setpoint)) {
			_rates_setpoint(0) = PX4_ISFINITE(vehicle_rates_setpoint.roll)  ? vehicle_rates_setpoint.roll  : rates(0);
			_rates_setpoint(1) = PX4_ISFINITE(vehicle_rates_setpoint.pitch) ? vehicle_rates_setpoint.pitch : rates(1);
			_rates_setpoint(2) = PX4_ISFINITE(vehicle_rates_setpoint.yaw)   ? vehicle_rates_setpoint.yaw   : rates(2);
			_thrust_setpoint = Vector3f(vehicle_rates_setpoint.thrust_body);
		}

		const int32_t indi_mode = _param_mc_indi_mode.get();
		const bool indi_shadow = indi_mode == 2;
		// In shadow mode INDI is computed and logged but never drives the vehicle,
		// so it does not need offboard and always "uses" the CTBR path (it simply
		// reads the same _rates_setpoint the stock PID is about to act on).
		const bool indi_requested = indi_mode == 1 || indi_shadow;
		const bool indi_allowed = indi_requested
					  && (indi_shadow || !_param_mc_indi_only_of.get()
					      || _vehicle_control_mode.flag_control_offboard_enabled);
		bool use_indi = false;

		if (indi_allowed) {
			if (!indi_shadow && _param_mc_indi_sp_mode.get() == 0) {
				if (_vehicle_attitude.timestamp != 0) {
					const Quatf q{_vehicle_attitude.q};

					if (updateAccelHeadingFrontend(q, _rates_setpoint, _thrust_setpoint, angular_accel_ff, indi_q_sp,
							      indi_accel_sp, indi_heading_sp)) {
						indi_setpoint_source = indi_status_s::SOURCE_ACCEL_HEADING;
						use_indi = true;
						_indi_last_q_sp = indi_q_sp;
						_indi_last_accel_sp = indi_accel_sp;
						_indi_last_heading_sp = indi_heading_sp;

						vehicle_rates_setpoint.roll = _rates_setpoint(0);
						vehicle_rates_setpoint.pitch = _rates_setpoint(1);
						vehicle_rates_setpoint.yaw = _rates_setpoint(2);
						_thrust_setpoint.copyTo(vehicle_rates_setpoint.thrust_body);
						vehicle_rates_setpoint.timestamp = hrt_absolute_time();
						_vehicle_rates_setpoint_pub.publish(vehicle_rates_setpoint);

					} else {
						indi_flags |= indi_status_s::FLAG_SP_TIMEOUT;
					}

				} else {
					indi_flags |= indi_status_s::FLAG_SP_TIMEOUT;
				}

			} else {
				indi_setpoint_source = indi_status_s::SOURCE_CTBR;
				use_indi = true;
			}
		}

		if (indi_requested && !use_indi) {
			indi_flags |= indi_status_s::FLAG_FALLBACK;
			_indi_filters_initialized = false;
		}

		// run the rate controller
		if (_vehicle_control_mode.flag_control_rates_enabled) {

			const bool armed_rotary = _vehicle_control_mode.flag_armed
						  && _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

			if (!armed_rotary) {
				_rate_control.resetIntegral();
				_indi_actuator_model.reset();
				_indi_published_torque_valid = false;
				_indi_fallback_latched = false;
				_indi_fallback_reason = indi_status_s::FALLBACK_NONE;
				_indi_active_prev = false;
				_indi_dynamic_model_ready = false;
			}

			if (indi_mode != 1) {
				_indi_fallback_latched = false;
				_indi_fallback_reason = indi_status_s::FALLBACK_NONE;
			}

			if (indi_mode == 0) {
				_indi_dynamic_model_ready = false;
			}

			// update saturation status from control allocation feedback
			control_allocator_status_s control_allocator_status;

			if (_control_allocator_status_sub.update(&control_allocator_status)) {
				_allocator_unallocated_torque = Vector3f(control_allocator_status.unallocated_torque);
				_allocator_status_timestamp = control_allocator_status.timestamp;
				Vector<bool, 3> saturation_positive;
				Vector<bool, 3> saturation_negative;

				if (!control_allocator_status.torque_setpoint_achieved) {
					for (size_t i = 0; i < 3; i++) {
						if (control_allocator_status.unallocated_torque[i] > FLT_EPSILON) {
							saturation_positive(i) = true;

						} else if (control_allocator_status.unallocated_torque[i] < -FLT_EPSILON) {
							saturation_negative(i) = true;
						}
					}
				}

				// TODO: send the unallocated value directly for better anti-windup
				_rate_control.setSaturationStatus(saturation_positive, saturation_negative);
			}

			const bool allocator_feedback_valid = _allocator_status_timestamp != 0
							      && hrt_elapsed_time(&_allocator_status_timestamp) < 50_ms;
			updateIndiCommandHistory(allocator_feedback_valid);
			_actuator_motors_sub.update(&_actuator_motors);

			const Vector3f base_rate_sp{_rates_setpoint};
			const Vector3f rate_sysid_injection = updateRateSysid(angular_velocity.timestamp_sample, rates,
										 base_rate_sp, armed_rotary,
										 allocator_feedback_valid);
			const Vector3f rate_sp_for_control = base_rate_sp + rate_sysid_injection;

			Vector3f omega_dot_raw_used{angular_accel};
			Vector3f omega_dot_f{};
			Vector3f nu{};
			Vector3f nu_comp{};
			Vector3f tau0_f{};
			Vector3f tau_comp{};
			Vector3f tau_phys{};
			Vector3f tau_norm{};
			Vector3f att_control_pid = _rate_control.update(rates, rate_sp_for_control, angular_accel, dt,
								      _maybe_landed || _landed);
			_rate_sysid_pid_torque = att_control_pid;
			Vector3f att_control = att_control_pid + _rate_sysid_torque_injection;

			for (int axis = 0; axis < 3; ++axis) {
				att_control(axis) = math::constrain(att_control(axis), -1.f, 1.f);
			}

			const bool indi_takeover_requested = use_indi && armed_rotary && !indi_shadow;
			const bool indi_shadow_run = use_indi && armed_rotary && indi_shadow;
			bool indi_driving = false;

			if (indi_takeover_requested || indi_shadow_run) {
				if (!_indi_tau_norm_prev_valid || (indi_takeover_requested && !_indi_active_prev)) {
					_indi_tau_norm_prev = att_control_pid;
					_indi_tau_norm_prev_valid = true;
					_indi_tau_phys_applied_prev = att_control_pid.emult(_indi_torque_max);
					_indi_command_increment_prev = att_control_pid - _indi_actuator_state_f;
				}

				const Vector3f indi_candidate = updateIndiControl(angular_velocity.timestamp_sample, rates,
										   _rates_setpoint, angular_accel, angular_accel_ff, dt,
										   _allocator_unallocated_torque, allocator_feedback_valid,
										   indi_flags, omega_dot_raw_used, omega_dot_f, nu, nu_comp,
										   tau0_f, tau_comp, tau_phys, tau_norm);
				updateIndiFallback(indi_candidate, dt, indi_takeover_requested, indi_flags);

				if (indi_takeover_requested && !_indi_fallback_latched
				    && !(indi_flags & (indi_status_s::FLAG_MODEL_INVALID | indi_status_s::FLAG_ACTUATOR_STALE))) {
					const int32_t axis_mask = _param_mc_indi_axes.get() & 7;

					for (int axis = 0; axis < 3; ++axis) {
						if (axis_mask & (1 << axis)) {
							att_control(axis) = indi_candidate(axis);
							indi_driving = true;
						}
					}
				}

			} else {
				tau_norm = att_control;
			}

			if (indi_takeover_requested && !indi_driving) {
				indi_flags |= indi_status_s::FLAG_FALLBACK;
			}

			if (indi_driving) {
				tau_norm = att_control;
				tau_phys = tau_norm.emult(_indi_torque_max);
				_indi_tau_norm_prev = tau_norm;
				_indi_command_increment_prev = tau_norm - _indi_actuator_state_f;
			}

			_indi_active_prev = indi_driving;

			// publish rate controller status
			rate_ctrl_status_s rate_ctrl_status{};
			_rate_control.getRateControlStatus(rate_ctrl_status);
			rate_ctrl_status.timestamp = hrt_absolute_time();
			_controller_status_pub.publish(rate_ctrl_status);

			// publish thrust and torque setpoints
			vehicle_thrust_setpoint_s vehicle_thrust_setpoint{};
			vehicle_torque_setpoint_s vehicle_torque_setpoint{};

			_thrust_setpoint.copyTo(vehicle_thrust_setpoint.xyz);
			vehicle_torque_setpoint.xyz[0] = PX4_ISFINITE(att_control(0)) ? att_control(0) : 0.f;
			vehicle_torque_setpoint.xyz[1] = PX4_ISFINITE(att_control(1)) ? att_control(1) : 0.f;
			vehicle_torque_setpoint.xyz[2] = PX4_ISFINITE(att_control(2)) ? att_control(2) : 0.f;

			// Scale stock PID setpoints by battery status if enabled. INDI uses angular acceleration feedback to
			// adapt to actuator effectiveness, so applying this extra scale would desynchronize tau0 feedback.
			if (_param_mc_bat_scale_en.get() && !indi_driving) {
				if (_battery_status_sub.updated()) {
					battery_status_s battery_status;

					if (_battery_status_sub.copy(&battery_status) && battery_status.connected && battery_status.scale > 0.f) {
						_battery_status_scale = battery_status.scale;
					}
				}

				if (_battery_status_scale > 0.f) {
					for (int i = 0; i < 3; i++) {
						vehicle_thrust_setpoint.xyz[i] = math::constrain(vehicle_thrust_setpoint.xyz[i] * _battery_status_scale, -1.f, 1.f);
						vehicle_torque_setpoint.xyz[i] = math::constrain(vehicle_torque_setpoint.xyz[i] * _battery_status_scale, -1.f, 1.f);
					}
				}
			}

			_rate_sysid_last_torque = Vector3f(vehicle_torque_setpoint.xyz);
			publishRateSysidStatus(angular_velocity.timestamp_sample, base_rate_sp, rate_sp_for_control, rates,
						      _rate_sysid_pid_torque, _rate_sysid_last_torque);

			// Keep INDI state primed from the PID output while PID is in charge so a
			// later takeover is bumpless — but NOT in shadow mode: there the shadow
			// updateIndiControl() call owns the INDI filters/tau0 and its tau_norm is
			// what we want to log, so overwriting it with the PID torque here would
			// both wipe the shadow state every sample and clobber the logged value.
			if (!indi_driving && !indi_shadow_run) {
				tau_norm = Vector3f(vehicle_torque_setpoint.xyz);
				const Vector3f pid_tau0 = tau_norm.emult(_indi_torque_max);
				resetIndiState(angular_accel, pid_tau0);
			}

			vehicle_thrust_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
			vehicle_thrust_setpoint.timestamp = hrt_absolute_time();
			_vehicle_thrust_setpoint_pub.publish(vehicle_thrust_setpoint);

			vehicle_torque_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
			vehicle_torque_setpoint.timestamp = hrt_absolute_time();
			_vehicle_torque_setpoint_pub.publish(vehicle_torque_setpoint);
			_indi_published_torque = Vector3f(vehicle_torque_setpoint.xyz);
			_indi_published_torque_timestamp = angular_velocity.timestamp_sample;
			_indi_published_torque_valid = armed_rotary;

			updateActuatorControlsStatus(vehicle_torque_setpoint, dt);

			publishIndiStatus(angular_velocity.timestamp_sample,
					  indi_driving ? indi_status_s::MODE_INDI
					  : (indi_shadow_run ? indi_status_s::MODE_SHADOW : indi_status_s::MODE_PID),
					  indi_setpoint_source,
					  indi_flags,
					  rate_sp_for_control,
					  rates,
					  omega_dot_raw_used,
					  omega_dot_f,
					  nu,
					  nu_comp,
					  tau0_f,
					  tau_comp,
					  tau_phys,
					  tau_norm,
					  _thrust_setpoint,
					  indi_q_sp,
					  indi_accel_sp,
					  indi_heading_sp);

		} else {
			if (rateSysidRunning()) {
				abortRateSysid(rate_sysid_status_s::ABORT_MODE, angular_velocity.timestamp_sample);
				publishRateSysidStatus(angular_velocity.timestamp_sample, _rates_setpoint, _rates_setpoint, rates,
							      _rate_sysid_pid_torque, _rate_sysid_last_torque);
			}
		}
	}

	perf_end(_loop_perf);
}

void MulticopterRateControl::updateActuatorControlsStatus(const vehicle_torque_setpoint_s &vehicle_torque_setpoint,
		float dt)
{
	for (int i = 0; i < 3; i++) {
		_control_energy[i] += vehicle_torque_setpoint.xyz[i] * vehicle_torque_setpoint.xyz[i] * dt;
	}

	_energy_integration_time += dt;

	if (_energy_integration_time > 500e-3f) {

		actuator_controls_status_s status;
		status.timestamp = vehicle_torque_setpoint.timestamp;

		for (int i = 0; i < 3; i++) {
			status.control_power[i] = _control_energy[i] / _energy_integration_time;
			_control_energy[i] = 0.f;
		}

		_actuator_controls_status_pub.publish(status);
		_energy_integration_time = 0.f;
	}
}

int MulticopterRateControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	MulticopterRateControl *instance = new MulticopterRateControl(vtol);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int MulticopterRateControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterRateControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the multicopter rate controller. It takes rate setpoints (in acro mode
via `manual_control_setpoint` topic) as inputs and outputs torque/thrust setpoints.

The default controller is the PX4 PID angular-rate loop. If MC_INDI_MODE is enabled,
the module can run the experimental INDI inner loop using either the normal CTBR
vehicle_rates_setpoint path or the dedicated indi_setpoint accel-heading interface.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_rate_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int mc_rate_control_main(int argc, char *argv[])
{
	return MulticopterRateControl::main(argc, argv);
}

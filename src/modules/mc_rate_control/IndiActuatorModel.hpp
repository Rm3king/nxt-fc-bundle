/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be used to
 *    endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 ****************************************************************************/

#pragma once

#include <matrix/matrix/math.hpp>

#include <cmath>
#include <cstdint>

class IndiActuatorModel
{
public:
	static constexpr size_t kHistoryLength{64};

	void reset()
	{
		_count = 0;
		_next = 0;
		_state.zero();
		_state_initialized = false;
	}

	void push(uint64_t timestamp_sample, const matrix::Vector3f &command)
	{
		if (timestamp_sample == 0 || !command.isAllFinite()) {
			return;
		}

		if (_count > 0) {
			const size_t newest = (_next + kHistoryLength - 1) % kHistoryLength;

			if (timestamp_sample < _history[newest].timestamp_sample) {
				return;
			}

			if (timestamp_sample == _history[newest].timestamp_sample) {
				_history[newest].command = command;
				return;
			}
		}

		_history[_next] = {timestamp_sample, command};
		_next = (_next + 1) % kHistoryLength;
		_count = _count < kHistoryLength ? _count + 1 : kHistoryLength;
	}

	bool sample(uint64_t query_timestamp, matrix::Vector3f &command) const
	{
		if (_count == 0) {
			return false;
		}

		const size_t oldest = (_next + kHistoryLength - _count) % kHistoryLength;
		const size_t newest = (_next + kHistoryLength - 1) % kHistoryLength;

		// A delayed query older than the available history cannot be aligned to the
		// gyro sample. Do not silently substitute the oldest command: that removes
		// the configured pure delay exactly during controller start-up.
		if (query_timestamp < _history[oldest].timestamp_sample) {
			return false;
		}

		if (query_timestamp == _history[oldest].timestamp_sample) {
			command = _history[oldest].command;
			return true;
		}

		if (query_timestamp >= _history[newest].timestamp_sample) {
			command = _history[newest].command;
			return true;
		}

		for (size_t offset = 1; offset < _count; ++offset) {
			const size_t upper_index = (oldest + offset) % kHistoryLength;
			const size_t lower_index = (oldest + offset - 1) % kHistoryLength;
			const Sample &lower = _history[lower_index];
			const Sample &upper = _history[upper_index];

			if (query_timestamp <= upper.timestamp_sample) {
				const uint64_t span = upper.timestamp_sample - lower.timestamp_sample;
				const float ratio = span > 0 ? static_cast<float>(query_timestamp - lower.timestamp_sample) / span : 0.f;
				command = lower.command + (upper.command - lower.command) * ratio;
				return true;
			}
		}

		return false;
	}

	bool update(uint64_t timestamp_sample, float dt, float delay_s, float time_constant_s,
		    matrix::Vector3f &state)
	{
		const uint64_t delay_us = static_cast<uint64_t>(fmaxf(delay_s, 0.f) * 1e6f);
		const uint64_t query_timestamp = timestamp_sample > delay_us ? timestamp_sample - delay_us : 0;
		matrix::Vector3f delayed_command{};

		if (!sample(query_timestamp, delayed_command)) {
			return false;
		}

		if (!_state_initialized) {
			_state = delayed_command;
			_state_initialized = true;

		} else {
			const float tau = fmaxf(time_constant_s, 1e-4f);
			const float alpha = expf(-fmaxf(dt, 0.f) / tau);
			_state = alpha * _state + (1.f - alpha) * delayed_command;
		}

		state = _state;
		return state.isAllFinite();
	}

	const matrix::Vector3f &state() const { return _state; }
	bool initialized() const { return _state_initialized; }
	uint64_t newestTimestamp() const
	{
		return _count > 0 ? _history[(_next + kHistoryLength - 1) % kHistoryLength].timestamp_sample : 0;
	}

private:
	struct Sample {
		uint64_t timestamp_sample{0};
		matrix::Vector3f command{};
	};

	Sample _history[kHistoryLength] {};
	size_t _count{0};
	size_t _next{0};
	matrix::Vector3f _state{};
	bool _state_initialized{false};
};

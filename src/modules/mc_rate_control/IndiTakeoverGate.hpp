/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <cstdint>

class IndiTakeoverGate
{
public:
	static constexpr uint64_t kAirborneDwellUs{1'000'000};

	void reset() { _airborne_since = 0; }

	void update(uint64_t timestamp, bool landed, bool maybe_landed, bool ground_contact)
	{
		if (landed || maybe_landed || ground_contact) {
			reset();

		} else if (_airborne_since == 0 && timestamp != 0) {
			_airborne_since = timestamp;
		}
	}

	bool ready(uint64_t timestamp) const
	{
		return _airborne_since != 0
		       && timestamp >= _airborne_since
		       && timestamp - _airborne_since > kAirborneDwellUs;
	}

private:
	uint64_t _airborne_since{0};
};

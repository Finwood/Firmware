/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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

#include "SDP3X.hpp"

/**
 * @file SDP3X.hpp
 *
 * Driver for Sensirion SDP3X Differential Pressure Sensor
 *
 */

int
SDP3X::probe()
{
	return !init_sdp3x();
}

int SDP3X::write_command(uint16_t command)
{
	uint8_t cmd[2];
	cmd[0] = static_cast<uint8_t>(command >> 8);
	cmd[1] = static_cast<uint8_t>(command & 0xff);
	return transfer(&cmd[0], 2, nullptr, 0);
}

bool
SDP3X::init_sdp3x()
{
	// step 1 - reset on broadcast
	uint16_t prev_addr = get_address();
	set_address(0);
	uint8_t reset_cmd = 0x06;
	(void)transfer(&reset_cmd, 1, nullptr, 0);
	set_address(prev_addr);
	usleep(20000);

	// step 2 - configure
	int ret = write_command(0x3615);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		PX4_ERR("config failed");
		return false;
	}

	usleep(8000);

	// step 3 - identify sensor type
	// ret = write_command(0x367c);
	// ret = write_command(0xe102);

	// uint8_t cmd[4];
	// uint8_t val[18];

	// uint8_t cmd[4];
	// cmd[0] = static_cast<uint8_t>(0x367c >> 8);
	// cmd[1] = static_cast<uint8_t>(0x367c & 0xff);
	// cmd[2] = static_cast<uint8_t>(0xe102 >> 8);
	// cmd[3] = static_cast<uint8_t>(0xe102 & 0xff);
	// return transfer(&cmd[0], 4, &val[0], 18);

	// if (ret == PX4_OK) {
	// 	PX4_INFO("request ID value cmd: %X ret: %02X%02X%02X%02X", (unsigned char)cmd[0],
	// 		 (unsigned char)val[0], (unsigned char)val[1], (unsigned char)val[3], (unsigned char)val[4]);

	// } else {
	// 	perf_count(_comms_errors);
	// 	return false;
	// }

	return true;
}

int
SDP3X::collect()
{
	perf_begin(_sample_perf);

	// read 9 bytes from the sensor
	uint8_t val[9];
	int ret = transfer(nullptr, 0, &val[0], 9);

	// if (ret != PX4_OK) {
	// 	perf_count(_comms_errors);
	// 	return ret;
	// }

	// Check the CRC
	if (!crc(&val[0], 2, val[2]) || !crc(&val[3], 2, val[5]) || !crc(&val[6], 2, val[8])) {
		perf_count(_comms_errors);
		return EAGAIN;

	} else {
		ret = 0;
	}

	int16_t P = (((uint16_t)val[0]) << 8) | val[1];
	int16_t temp = (((uint16_t)val[3]) << 8) | val[4];
	int16_t scale = (((uint16_t)val[6]) << 8) | val[7];

	float diff_press_pa_raw = fabsf(static_cast<float>(P) / static_cast<float>(scale));
	float temperature_c = temp / static_cast<float>(200.0f);

	//PX4_WARN("T: %.1f PSI: %.3f Pa: %.3f", (double)temperature_c, (double)diff_press_PSI, (double)diff_press_pa_raw);

	// the raw value still should be compensated for the known offset
	//diff_press_pa_raw -= _diff_pres_offset;

	differential_pressure_s report;

	// track maximum differential pressure measured (so we can work out top speed).
	if (diff_press_pa_raw > _max_differential_pressure_pa) {
		_max_differential_pressure_pa = diff_press_pa_raw;
	}

	report.timestamp = hrt_absolute_time();
	report.error_count = perf_event_count(_comms_errors);
	report.temperature = temperature_c;
	report.differential_pressure_filtered_pa = _filter.apply(diff_press_pa_raw);
	report.differential_pressure_raw_pa = diff_press_pa_raw;
	report.max_differential_pressure_pa = _max_differential_pressure_pa;

	if (_airspeed_pub != nullptr && !(_pub_blocked)) {
		// publish it
		orb_publish(ORB_ID(differential_pressure), _airspeed_pub, &report);
	}

	new_report(report);

	// notify anyone waiting for data
	poll_notify(POLLIN);

	ret = OK;

	perf_end(_sample_perf);

	return ret;
}

void
SDP3X::cycle()
{
	int ret = PX4_ERROR;

	// measurement phase
	ret = collect();

	if (PX4_OK != ret) {
		_sensor_ok = false;
		DEVICE_DEBUG("measure error");
	}

	// schedule a fresh cycle call when the measurement is done
	work_queue(HPWORK, &_work, (worker_t)&Airspeed::cycle_trampoline, this, USEC2TICK(CONVERSION_INTERVAL));
}

bool SDP3X::crc(const uint8_t data[], unsigned size, uint8_t checksum)
{
	uint8_t crc = 0xff;

	// calculate 8-bit checksum with polynomial 0x31 (x^8 + x^5 + x^4 + 1)
	for (unsigned i = 0; i < size; i++) {
		crc ^= (data[i]);

		for (int bit = 8; bit > 0; --bit) {
			if (crc & 0x80) {
				crc = (crc << 1) ^ 0x31;

			} else {
				crc = (crc << 1);
			}
		}
	}

	// verify checksum
	return (crc == checksum);
}

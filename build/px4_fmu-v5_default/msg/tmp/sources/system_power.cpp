/****************************************************************************
 *
 *   Copyright (C) 2013-2016 PX4 Development Team. All rights reserved.
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

/* Auto-generated by genmsg_cpp from file system_power.msg */


#include <inttypes.h>
#include <px4_log.h>
#include <px4_defines.h>
#include <uORB/topics/system_power.h>
#include <drivers/drv_hrt.h>
#include <lib/drivers/device/Device.hpp>

constexpr char __orb_system_power_fields[] = "uint64_t timestamp;float voltage5v_v;float voltage3v3_v;uint8_t v3v3_valid;uint8_t usb_connected;uint8_t brick_valid;uint8_t usb_valid;uint8_t servo_valid;uint8_t periph_5v_oc;uint8_t hipower_5v_oc;uint8_t[1] _padding0;";

ORB_DEFINE(system_power, struct system_power_s, 23, __orb_system_power_fields);


void print_message(const system_power_s& message)
{

	PX4_INFO_RAW(" system_power_s\n");
	if (message.timestamp != 0) {
		PX4_INFO_RAW("\ttimestamp: %" PRIu64 "  (%.6f seconds ago)\n", message.timestamp, hrt_elapsed_time(&message.timestamp) / 1e6);
	} else {
		PX4_INFO_RAW("\n");
	}
	PX4_INFO_RAW("\tvoltage5v_v: %.4f\n", (double)message.voltage5v_v);
	PX4_INFO_RAW("\tvoltage3v3_v: %.4f\n", (double)message.voltage3v3_v);
	PX4_INFO_RAW("\tv3v3_valid: %u\n", message.v3v3_valid);
	PX4_INFO_RAW("\tusb_connected: %u\n", message.usb_connected);
	PX4_INFO_RAW("\tbrick_valid: %u\n", message.brick_valid);
	PX4_INFO_RAW("\tusb_valid: %u\n", message.usb_valid);
	PX4_INFO_RAW("\tservo_valid: %u\n", message.servo_valid);
	PX4_INFO_RAW("\tperiph_5v_oc: %u\n", message.periph_5v_oc);
	PX4_INFO_RAW("\thipower_5v_oc: %u\n", message.hipower_5v_oc);
	
}
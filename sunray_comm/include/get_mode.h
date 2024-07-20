/**
 * @brief PX4 custom mode constants
 * @file px4_custom_mode.h
 * PX4 custom flight modes
 *
 * Modifyed copy px4_custom_mode.h from PX4/Firmware
 *
 * @author Anton Babushkin <anton@px4.io>
 */
#include <array>
#include <unordered_map>
#include <stdexcept>
#include "px4_custom_mode.h"

#pragma once

//#include <stdint.h>

/* -*- mode stringify functions -*- */

typedef std::unordered_map<uint32_t, const std::string> cmode_map;
//! PX4 custom mode -> string
static const cmode_map px4_cmode_map{{
	{ px4::define_mode(px4::custom_mode::MAIN_MODE_MANUAL),           "MANUAL" },
	{ px4::define_mode(px4::custom_mode::MAIN_MODE_ACRO),             "ACRO" },
	{ px4::define_mode(px4::custom_mode::MAIN_MODE_ALTCTL),           "ALTCTL" },
	{ px4::define_mode(px4::custom_mode::MAIN_MODE_POSCTL),           "POSCTL" },
	{ px4::define_mode(px4::custom_mode::MAIN_MODE_OFFBOARD),         "OFFBOARD" },
	{ px4::define_mode(px4::custom_mode::MAIN_MODE_STABILIZED),       "STABILIZED" },
	{ px4::define_mode(px4::custom_mode::MAIN_MODE_RATTITUDE),        "RATTITUDE" },
	{ px4::define_mode_auto(px4::custom_mode::SUB_MODE_AUTO_MISSION), "AUTO.MISSION" },
	{ px4::define_mode_auto(px4::custom_mode::SUB_MODE_AUTO_LOITER),  "AUTO.LOITER" },
	{ px4::define_mode_auto(px4::custom_mode::SUB_MODE_AUTO_RTL),     "AUTO.RTL" },
	{ px4::define_mode_auto(px4::custom_mode::SUB_MODE_AUTO_LAND),    "AUTO.LAND" },
	{ px4::define_mode_auto(px4::custom_mode::SUB_MODE_AUTO_RTGS),    "AUTO.RTGS" },
	{ px4::define_mode_auto(px4::custom_mode::SUB_MODE_AUTO_READY),   "AUTO.READY" },
	{ px4::define_mode_auto(px4::custom_mode::SUB_MODE_AUTO_TAKEOFF), "AUTO.TAKEOFF" },
	{ px4::define_mode_auto(px4::custom_mode::SUB_MODE_AUTO_FOLLOW_TARGET), "AUTO.FOLLOW_TARGET" },
	{ px4::define_mode_auto(px4::custom_mode::SUB_MODE_AUTO_PRECLAND), "AUTO.PRECLAND" },
}};

static std::string str_mode_cmap(const cmode_map &cmap, uint32_t custom_mode)
{
	auto it = cmap.find(custom_mode);
	string str = "unknown";
	if (it != cmap.end())
		return it->second;
	else
		return str;
}

static bool cmode_from_str(std::string cmode_str, uint32_t &cmode)
{
	// 1. try find by name
	for (auto &mode : px4_cmode_map) {
		if (mode.second == cmode_str) {
			cmode = mode.first;
			return true;
		}
	}

	// 2. try convert integer
	//! @todo parse CMODE(dec)
	try {
		cmode = std::stoi(cmode_str, 0, 0);
		return true;
	}
	catch (std::invalid_argument &ex) {
		// failed
	}

	// Debugging output.
	std::ostringstream os;
	for (auto &mode : px4_cmode_map)
		os << " " << mode.second;

	ROS_ERROR_STREAM_NAMED("uas", "MODE: Unknown mode: " << cmode_str);
	ROS_INFO_STREAM_NAMED("uas", "MODE: Known modes are:" << os.str());

	return false;

}

static std::string get_mode(int base_mode, int custom_mode_int)
{
	px4::custom_mode custom_mode(custom_mode_int);

	// clear fields
	custom_mode.reserved = 0;
	if (custom_mode.main_mode != px4::custom_mode::MAIN_MODE_AUTO) 
    {
		ROS_WARN_COND_NAMED(custom_mode.sub_mode != 0, "uas", "PX4: Unknown sub-mode %d.%d",
                            custom_mode.main_mode, custom_mode.sub_mode);
		custom_mode.sub_mode = 0;
	}

	return str_mode_cmap(px4_cmode_map, custom_mode.data);

}
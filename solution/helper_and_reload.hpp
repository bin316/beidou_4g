/*
 * helper_and_reload.hpp
 *
 *  Created on: Feb 12, 2025
 *      Author: IRIS
 */

#ifndef HELPER_AND_RELOAD_HPP_
#define HELPER_AND_RELOAD_HPP_

#include "utilties.h"
#include "Protocol.h"
#include "Solution.h"
#include "main.h"

/*
 * @func helper_status_byte_maker
 *
 * @brief make a status byte with the given parameters
 * @param mode: the solution mode
 * @param wake: the wake source
 * @param vib: vibration occured
 * @param leaned: leaned
 * @param geoStat: gnss status
 * @return the status byte
 */
uint8_t helper_status_byte_maker(solution_mode_e mode,
	util_lowpower_wake_source_e wake, bool vib, bool leaned, bool geoStat)
    {
    uint8_t status = 0;
//	7		6		5		4		3		2		1		0
//	|mode   	|wake				|vib		|lean		|tmNew		|gnssNew
    switch (mode)
	{
    case solution_mode_e::wm_factory:
	CLEAR_BIT(status, 0x80);
	CLEAR_BIT(status, 0x40);
	break;
    case solution_mode_e::wm_idle:
	CLEAR_BIT(status, 0x80);
	SET_BIT(status, 0x40);
    case solution_mode_e::wm_work:
	SET_BIT(status, 0x80);
	CLEAR_BIT(status, 0x40);
    default:
	break;
	}
    switch (wake)
	{
    case util_lowpower_wake_source_e::pin:
	CLEAR_BIT(status, 0x20);
	CLEAR_BIT(status, 0x10);
	break;
    case util_lowpower_wake_source_e::rtc:
	CLEAR_BIT(status, 0x20);
	SET_BIT(status, 0x10);
	break;
    case util_lowpower_wake_source_e::regular:
	SET_BIT(status, 0x20);
	CLEAR_BIT(status, 0x10);
	break;
	}
    if (vib)
	SET_BIT(status, 0x08);
    if (leaned)
	SET_BIT(status, 0x02);
    if (geoStat)
	SET_BIT(status, 0x01);
    return status;
    }






/*
 * @func <password == reload>
 *
 * @brief compare two password object
 * 		return true if they are the same
 */
bool operator ==(password &dest, password &source)
    {
    for (size_t i = 0; i < sizeof(password::item); i++)
	{
	if (dest.item[i] != source.item[i])
	    return false;
	}
    return true;
    }

/*
 * @func <pb_code == reload>
 * @brief compare two code object
 * 		return true if they are the same
 */
bool operator ==(pb_code &dest, pb_code &source)
    {
    return dest.index == source.index && dest.major == source.major
	    && dest.minor == source.minor;
    }

#endif /* HELPER_AND_RELOAD_HPP_ */

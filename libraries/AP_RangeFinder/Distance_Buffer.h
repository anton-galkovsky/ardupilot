/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include "RangeFinder.h"

#define DISTANCE_BUFFER_LENGTH 4

class Distance_Buffer {

    static int compare(const void * a, const void * b) {
        return *((uint16_t*)a) - *((uint16_t*)b);
    }

	const AP_RangeFinder_Backend *sensor;

	uint16_t buf[DISTANCE_BUFFER_LENGTH] {};
	uint16_t sorted_buf[DISTANCE_BUFFER_LENGTH] {};
	uint8_t next_ind;
	uint32_t last_measure_time_ms;
	uint32_t penult_measure_time_ms;
	uint16_t last_value_cm;
	uint16_t penult_value_cm;
	float value_derivative_cmms;

	void add_value(uint16_t val, uint32_t sensor_last_reading_ms);

public:

	Distance_Buffer(AP_RangeFinder_Backend *sensor);

	void add_sensor_data(uint32_t time);

	//return median
	uint16_t get_dist_cm() const { return last_value_cm; }

	uint16_t get_derivative_cmms() const { return value_derivative_cmms;	}

	uint32_t get_last_measure_time_ms() const { return last_measure_time_ms; }
};

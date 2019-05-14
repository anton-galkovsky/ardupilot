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

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "RangeFinder_Backend.h"
#include "Distance_Buffer.h"

#include <GCS_MAVLink/GCS.h>
#include <stdio.h>

Distance_Buffer::Distance_Buffer(AP_RangeFinder_Backend *sens) {
	next_ind = 0;
	last_measure_time_ms = 1;
	penult_measure_time_ms = 0;
	last_value_cm = 0;
	penult_value_cm = 0;
	value_derivative_cmms = 0.0f;
	sensor = sens;
}

void Distance_Buffer::add_value(uint16_t val, uint32_t sensor_last_reading_ms) {
	penult_measure_time_ms = last_measure_time_ms;
	last_measure_time_ms = sensor_last_reading_ms;

	buf[next_ind] = val;
	next_ind = (next_ind + 1) % DISTANCE_BUFFER_LENGTH;

	memcpy(sorted_buf, buf, DISTANCE_BUFFER_LENGTH * sizeof(uint16_t));
	qsort(sorted_buf, DISTANCE_BUFFER_LENGTH, sizeof(uint16_t), compare);

	penult_value_cm = last_value_cm;
	last_value_cm = sorted_buf[DISTANCE_BUFFER_LENGTH / 2];

	value_derivative_cmms = (last_value_cm - penult_value_cm)
			/ (last_measure_time_ms - penult_measure_time_ms);
}

void Distance_Buffer::add_sensor_data(uint32_t time) {
	uint32_t sensor_last_reading_ms = sensor->last_reading_ms();
	if (time - sensor_last_reading_ms > 30) {
	    sensor_last_reading_ms = time;
	}
	if (last_measure_time_ms == sensor_last_reading_ms) {
		return;
	}

	uint16_t sensor_data;
	if (sensor->has_data()) {
		sensor_data = sensor->distance_cm();
	} else {
		sensor_data = sensor->max_distance_cm() - 1;
	}
	add_value(sensor_data, sensor_last_reading_ms);
}


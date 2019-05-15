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

	for (uint8_t i = 0; i < DISTANCE_BUFFER_LENGTH; i++) {
		sorted_buf[i] = buf[i];
	}
	uint16_t x;
	for (uint8_t i = 0; i < DISTANCE_BUFFER_LENGTH - 1; i++) {
		if (sorted_buf[i] > sorted_buf[i + 1]) {
			x = sorted_buf[i];
			sorted_buf[i] = sorted_buf[i + 1];
			sorted_buf[i + 1] = x;
			if (i != 0) {
				i -= 2;
			}
		}
	}

	penult_value_cm = last_value_cm;
	last_value_cm = sorted_buf[DISTANCE_BUFFER_LENGTH / 2];

	value_derivative_cmms = 1.0f * (last_value_cm - penult_value_cm)
			/ (last_measure_time_ms - penult_measure_time_ms);
}

void Distance_Buffer::send_data_to_gcs() const {
	char str[15 + DISTANCE_BUFFER_LENGTH * 4];
	char loc[5];
	strcpy(str, "dist_buf: ");
	for (uint8_t i = 0; i < DISTANCE_BUFFER_LENGTH; i++) {
		sprintf(loc, "%03d ", sorted_buf[i]);
		strcat(str, loc);
	}
	gcs().send_text(MAV_SEVERITY_CRITICAL, str);
}

void Distance_Buffer::add_sensor_data(uint32_t time) {
	uint32_t sensor_last_reading_ms = sensor->last_reading_ms();
	if (time - sensor_last_reading_ms > 100) {
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


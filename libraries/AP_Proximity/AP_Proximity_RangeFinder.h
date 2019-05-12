#pragma once

#include "AP_Proximity.h"
#include "AP_Proximity_Backend.h"

#include <AP_RangeFinder/RangeFinder.h>

#define PROXIMITY_RANGEFIDER_TIMEOUT_MS 200 // requests timeout after 0.2 seconds

#define DISTANCE_BUFFER_LENGTH 4

class distance_buf {

	float buf[DISTANCE_BUFFER_LENGTH] {};
	float sorted_buf[DISTANCE_BUFFER_LENGTH] {};
	uint8_t next_ind;
	bool actual_data;
	uint32_t last_measure_time;

    static int compare(const void * a, const void * b) {
        return *((float*)a) - *((float*)b);
    }

public:

	distance_buf() {
		next_ind = 0;
		actual_data = true;
		last_measure_time = 0;
	}

	void add_value(float val, uint32_t time) {
		if (last_measure_time == time) {
			return;
		}
		last_measure_time = time;
		buf[next_ind] = val;
		next_ind = (next_ind + 1) % DISTANCE_BUFFER_LENGTH;
		actual_data = false;
	}

	float get_median() {
		if (!actual_data) {
			memcpy(sorted_buf, buf, DISTANCE_BUFFER_LENGTH * sizeof(float));
		    qsort(sorted_buf, DISTANCE_BUFFER_LENGTH, sizeof(float), compare);
		    actual_data = true;
		}
	    return sorted_buf[DISTANCE_BUFFER_LENGTH / 2];
	}

	uint32_t get_last_measure_time() {
		return last_measure_time;
	}
};

class AP_Proximity_RangeFinder : public AP_Proximity_Backend
{

public:
    // constructor
	AP_Proximity_RangeFinder(AP_Proximity &_frontend, AP_Proximity::Proximity_State &_state);

    // update state
    void update(void) override;

    // get maximum and minimum distances (in meters) of sensor
    float distance_max() const override { return _distance_max; }
    float distance_min() const override { return _distance_min; }

    // get distance upwards in meters. returns true on success
    bool get_upward_distance(float &distance) const override;

private:

    // horizontal distance support
    uint32_t _last_update_ms;   // system time of last RangeFinder reading
    float _distance_max;        // max range of sensor in meters
    float _distance_min;        // min range of sensor in meters

    // upward distance support
    uint32_t _last_upward_update_ms;    // system time of last update distance
    float _distance_upward;             // upward distance in meters, negative if the last reading was out of range

    uint32_t _last_downward_update_ms;
    float _distance_downward;

    distance_buf _horizontal_distance_bufs[PROXIMITY_SECTORS_MAX];
    distance_buf _upward_distance_buf;
    distance_buf _downward_distance_buf;
};

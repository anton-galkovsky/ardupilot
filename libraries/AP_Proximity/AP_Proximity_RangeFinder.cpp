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

#include <AP_HAL/AP_HAL.h>
#include "AP_Proximity_RangeFinder.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>
#include <stdio.h>
#include <AP_RangeFinder/RangeFinder_Backend.h>

#include <AP_RangeFinder/AP_RangeFinder_Params.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

AP_Proximity_RangeFinder::AP_Proximity_RangeFinder(AP_Proximity &_frontend,
                                   AP_Proximity::Proximity_State &_state) :
    AP_Proximity_Backend(_frontend, _state),
    _distance_upward(-1)
{
}

// update the state of the sensor
void AP_Proximity_RangeFinder::update(void)
{
    // exit immediately if no rangefinder object
    const RangeFinder *rngfnd = frontend.get_rangefinder();
    if (rngfnd == nullptr) {
        set_status(AP_Proximity::Proximity_NoData);
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Proximity rngfnd == nullptr");
        return;
    }

    uint32_t now = AP_HAL::millis();

    char str[60];     //for 6 instanses
    char loc[10];
    strcpy(str, "prox: ");


    // look through all rangefinders
    for (uint8_t i=0; i < rngfnd->num_sensors(); i++) {
        AP_RangeFinder_Backend *sensor = rngfnd->get_backend(i);
        if (sensor == nullptr) {
            continue;
        }

        float distance_min_m = sensor->min_distance_cm() / 100.0f;
        float distance_max_m = sensor->max_distance_cm() / 100.0f;
        float distance = sensor->get_smooth_buf_dist_cm() / 100.0f;
//        float distance = sensor->distance_cm() / 100.0f;

//        sprintf(loc, "%2.2f ", (double)(sensor->get_dist_derivative_cmms()));

        // check for horizontal range finders
        if (sensor->orientation() <= ROTATION_YAW_315) {
            uint8_t sector = (uint8_t)sensor->orientation();


            _angle[sector] = sector * 45;
            _distance_min = distance_min_m;
            _distance_max = distance_max_m;

            _distance[sector] = distance;

            _dist_derivative[sector] = sensor->get_dist_derivative_cmms() * 10.0f;

           	sprintf(loc, "%02.4f ", _dist_derivative[sector]);

            _distance_valid[sector] = (_distance[sector] >= _distance_min) && (_distance[sector] <= _distance_max);
            _last_update_ms = now;
            update_boundary_for_sector(sector);
        }
        // check upward facing range finder
        else if (sensor->orientation() == ROTATION_PITCH_90) {
            _distance_upward = distance;

//        	sprintf(loc, "%d ", (int)(_distance_upward * 100.0f));

            if ((_distance_upward < distance_min_m) || (_distance_upward > distance_max_m)) {
                _distance_upward = -1.0; // mark an valid reading
            }
            _last_upward_update_ms = now;
        }
        // check downward facing range finder
        else if (sensor->orientation() == ROTATION_PITCH_270) {
           	_distance_downward = distance;
           	_distance_downward_der = sensor->get_dist_derivative_cmms() * 10.0f;

           	sprintf(loc, "%02.4f ", _distance_downward_der);

            if ((_distance_downward < distance_min_m) || (_distance_downward > distance_max_m)) {
                _distance_downward = -1.0; // mark an valid reading
            }
            _last_downward_update_ms = now;
        }
       	strcat(str, loc);
    }
//    gcs().send_text(MAV_SEVERITY_CRITICAL, str);

//    gcs().send_text(MAV_SEVERITY_CRITICAL, "prox: %4d",
//			rc().get_radio_in(5));

    // check for timeout and set health status
    if ((_last_update_ms == 0) || (now - _last_update_ms > PROXIMITY_RANGEFIDER_TIMEOUT_MS)) {
        set_status(AP_Proximity::Proximity_NoData);
    	gcs().send_text(MAV_SEVERITY_CRITICAL, "Proximity_NoData");
    } else {
        set_status(AP_Proximity::Proximity_Good);
    }
}

// get distance upwards in meters. returns true on success
bool AP_Proximity_RangeFinder::get_upward_distance(float &distance) const
{
    if ((AP_HAL::millis() - _last_upward_update_ms <= PROXIMITY_RANGEFIDER_TIMEOUT_MS) &&
        is_positive(_distance_upward)) {
        distance = _distance_upward;
        return true;
    }
    return false;
}

bool AP_Proximity_RangeFinder::get_downward_distance(float &distance) const
{
   if ((AP_HAL::millis() - _last_downward_update_ms <= PROXIMITY_RANGEFIDER_TIMEOUT_MS) &&
        is_positive(_distance_downward)) {
        distance = _distance_downward;
        return true;
    }
    return false;
}

bool AP_Proximity_RangeFinder::get_downward_dist_der(float &dist_der) const
{
   if ((AP_HAL::millis() - _last_downward_update_ms <= PROXIMITY_RANGEFIDER_TIMEOUT_MS) &&
        is_positive(_distance_downward)) {
        dist_der = _distance_downward_der;
        return true;
    }
    return false;
}

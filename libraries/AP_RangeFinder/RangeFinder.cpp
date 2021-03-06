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

#include "RangeFinder.h"
#include "AP_RangeFinder_analog.h"
#include "AP_RangeFinder_PulsedLightLRF.h"
#include "AP_RangeFinder_MaxsonarI2CXL.h"
#include "AP_RangeFinder_MaxsonarSerialLV.h"
#include "AP_RangeFinder_BBB_PRU.h"
#include "AP_RangeFinder_LightWareI2C.h"
#include "AP_RangeFinder_LightWareSerial.h"
#if (CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP || \
     CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO) &&      \
    defined(HAVE_LIBIIO)
#include "AP_RangeFinder_Bebop.h"
#endif
#include "AP_RangeFinder_MAVLink.h"
#include "AP_RangeFinder_LeddarOne.h"
#include "AP_RangeFinder_uLanding.h"
#include "AP_RangeFinder_TeraRangerI2C.h"
#include "AP_RangeFinder_VL53L0X.h"
#include "AP_RangeFinder_VL53L1X.h"
#include "AP_RangeFinder_NMEA.h"
#include "AP_RangeFinder_Wasp.h"
#include "AP_RangeFinder_Benewake.h"
#include "AP_RangeFinder_PWM.h"
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

#include "AP_ServoRelayEvents/AP_ServoRelayEvents.h"
#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>
#include <stdio.h>
#include <../ArduCopter/Copter.h>

extern const AP_HAL::HAL &hal;

// table of user settable parameters
const AP_Param::GroupInfo RangeFinder::var_info[] = {

	// @Group: 1_
	// @Path: AP_RangeFinder_Params.cpp
	AP_SUBGROUPINFO_FLAGS(params[0], "1_", 25, RangeFinder, AP_RangeFinder_Params, AP_PARAM_FLAG_IGNORE_ENABLE),

    // @Group: 1_
    // @Path: AP_RangeFinder_Wasp.cpp
    AP_SUBGROUPVARPTR(drivers[0], "1_",  57, RangeFinder, backend_var_info[0]),

#if RANGEFINDER_MAX_INSTANCES > 1
    // @Group: 2_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[1], "2_", 27, RangeFinder, AP_RangeFinder_Params),

    // @Group: 2_
    // @Path: AP_RangeFinder_Wasp.cpp
    AP_SUBGROUPVARPTR(drivers[1], "2_",  58, RangeFinder, backend_var_info[1]),
#endif

#if RANGEFINDER_MAX_INSTANCES > 2
    // @Group: 3_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[2], "3_", 29, RangeFinder, AP_RangeFinder_Params),

    // @Group: 3_
    // @Path: AP_RangeFinder_Wasp.cpp
    AP_SUBGROUPVARPTR(drivers[2], "3_",  59, RangeFinder, backend_var_info[2]),
#endif

#if RANGEFINDER_MAX_INSTANCES > 3
    // @Group: 4_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[3], "4_", 31, RangeFinder, AP_RangeFinder_Params),

    // @Group: 4_
    // @Path: AP_RangeFinder_Wasp.cpp
    AP_SUBGROUPVARPTR(drivers[0], "4_",  60, RangeFinder, backend_var_info[3]),
#endif

#if RANGEFINDER_MAX_INSTANCES > 4
    // @Group: 5_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[4], "5_", 33, RangeFinder, AP_RangeFinder_Params),

    // @Group: 5_
    // @Path: AP_RangeFinder_Wasp.cpp
    AP_SUBGROUPVARPTR(drivers[4], "5_",  34, RangeFinder, backend_var_info[4]),
#endif

#if RANGEFINDER_MAX_INSTANCES > 5
    // @Group: 6_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[5], "6_", 35, RangeFinder, AP_RangeFinder_Params),

    // @Group: 6_
    // @Path: AP_RangeFinder_Wasp.cpp
    AP_SUBGROUPVARPTR(drivers[5], "6_",  36, RangeFinder, backend_var_info[5]),
#endif

#if RANGEFINDER_MAX_INSTANCES > 6
    // @Group: 7_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[6], "7_", 37, RangeFinder, AP_RangeFinder_Params),

    // @Group: 7_
    // @Path: AP_RangeFinder_Wasp.cpp
    AP_SUBGROUPVARPTR(drivers[6], "7_",  38, RangeFinder, backend_var_info[6]),
#endif

#if RANGEFINDER_MAX_INSTANCES > 7
    // @Group: 8_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[7], "8_", 39, RangeFinder, AP_RangeFinder_Params),

    // @Group: 8_
    // @Path: AP_RangeFinder_Wasp.cpp
    AP_SUBGROUPVARPTR(drivers[7], "8_",  40, RangeFinder, backend_var_info[7]),
#endif

#if RANGEFINDER_MAX_INSTANCES > 8
    // @Group: 9_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[8], "9_", 41, RangeFinder, AP_RangeFinder_Params),

    // @Group: 9_
    // @Path: AP_RangeFinder_Wasp.cpp
    AP_SUBGROUPVARPTR(drivers[8], "9_",  42, RangeFinder, backend_var_info[8]),
#endif

#if RANGEFINDER_MAX_INSTANCES > 9
    // @Group: A_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[9], "A_", 43, RangeFinder, AP_RangeFinder_Params),

    // @Group: A_
    // @Path: AP_RangeFinder_Wasp.cpp
    AP_SUBGROUPVARPTR(drivers[9], "A_",  44, RangeFinder, backend_var_info[9]),
#endif
    
    AP_GROUPEND
};

const AP_Param::GroupInfo *RangeFinder::backend_var_info[RANGEFINDER_MAX_INSTANCES];

RangeFinder::RangeFinder(AP_SerialManager &_serial_manager, enum Rotation orientation_default) :
    serial_manager(_serial_manager)
{
    AP_Param::setup_object_defaults(this, var_info);

    // set orientation defaults
    for (uint8_t i=0; i<RANGEFINDER_MAX_INSTANCES; i++) {
        params[i].orientation.set_default(orientation_default);
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("Rangefinder must be singleton");
    }
#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL
    _singleton = this;
}

void RangeFinder::convert_params(void) {
    if (params[0].type.configured_in_storage()) {
        // _params[0]._type will always be configured in storage after conversion is done the first time
        return;
    }

    struct ConversionTable {
        uint8_t old_element;
        uint8_t new_index;
        uint8_t instance;
    };

    const struct ConversionTable conversionTable[] = {
            {0, 0, 0}, //0, TYPE 1
            {1, 1, 0}, //1, PIN 1
            {2, 2, 0}, //2, SCALING 1
            {3, 3, 0}, //3, OFFSET 1
            {4, 4, 0}, //4, FUNCTION 1
            {5, 5, 0}, //5, MIN_CM 1
            {6, 6, 0}, //6, MAX_CM 1
            {7, 7, 0}, //7, STOP_PIN 1
            {8, 8, 0}, //8, SETTLE 1
            {9, 9, 0}, //9, RMETRIC 1
            {10, 10, 0}, //10, PWRRNG 1 (previously existed only once for all sensors)
            {11, 11, 0}, //11, GNDCLEAR 1
            {23, 12, 0}, //23, ADDR 1
            {49, 13, 0}, //49, POS 1
            {53, 14, 0}, //53, ORIENT 1

            //{57, 1, 0}, //57, backend 1

            {12, 0, 1}, //12, TYPE 2
            {13, 1, 1}, //13, PIN 2
            {14, 2, 1}, //14, SCALING 2
            {15, 3, 1}, //15, OFFSET 2
            {16, 4, 1}, //16, FUNCTION 2
            {17, 5, 1}, //17, MIN_CM 2
            {18, 6, 1}, //18, MAX_CM 2
            {19, 7, 1}, //19, STOP_PIN 2
            {20, 8, 1}, //20, SETTLE 2
            {21, 9, 1}, //21, RMETRIC 2
            //{, 10, 1}, //PWRRNG 2 (previously existed only once for all sensors)
            {22, 11, 1}, //22, GNDCLEAR 2
            {24, 12, 1}, //24, ADDR 2
            {50, 13, 1}, //50, POS 2
            {54, 14, 1}, //54, ORIENT 2

            //{58, 3, 1}, //58, backend 2

            {25, 0, 2}, //25, TYPE 3
            {26, 1, 2}, //26, PIN 3
            {27, 2, 2}, //27, SCALING 3
            {28, 3, 2}, //28, OFFSET 3
            {29, 4, 2}, //29, FUNCTION 3
            {30, 5, 2}, //30, MIN_CM 3
            {31, 6, 2}, //31, MAX_CM 3
            {32, 7, 2}, //32, STOP_PIN 3
            {33, 8, 2}, //33, SETTLE 3
            {34, 9, 2}, //34, RMETRIC 3
            //{, 10, 2}, //PWRRNG 3 (previously existed only once for all sensors)
            {35, 11, 2}, //35, GNDCLEAR 3
            {36, 12, 2}, //36, ADDR 3
            {51, 13, 2}, //51, POS 3
            {55, 14, 2}, //55, ORIENT 3

            //{59, 5, 2}, //59, backend 3

            {37, 0, 3}, //37, TYPE 4
            {38, 1, 3}, //38, PIN 4
            {39, 2, 3}, //39, SCALING 4
            {40, 3, 3}, //40, OFFSET 4
            {41, 4, 3}, //41, FUNCTION 4
            {42, 5, 3}, //42, MIN_CM 4
            {43, 6, 3}, //43, MAX_CM 4
            {44, 7, 3}, //44, STOP_PIN 4
            {45, 8, 3}, //45, SETTLE 4
            {46, 9, 3}, //46, RMETRIC 4
            //{, 10, 3}, //PWRRNG 4 (previously existed only once for all sensors)
            {47, 11, 3}, //47, GNDCLEAR 4
            {48, 12, 3}, //48, ADDR 4
            {52, 13, 3}, //52, POS 4
            {56, 14, 3}, //56, ORIENT 4

            //{60, 7, 3}, //60, backend 4
    };

    char param_name[17] = {0};
    AP_Param::ConversionInfo info;
    info.new_name = param_name;

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    info.old_key = 71;
#elif APM_BUILD_TYPE(APM_BUILD_ArduCopter)
    info.old_key = 53;
#elif APM_BUILD_TYPE(APM_BUILD_ArduSub)
    info.old_key = 35;
#elif APM_BUILD_TYPE(APM_BUILD_APMrover2)
    info.old_key = 197;
#else
    params[0].type.save(true);
    return; // no conversion is supported on this platform
#endif

    for (uint8_t i = 0; i < ARRAY_SIZE(conversionTable); i++) {
        uint8_t param_instance = conversionTable[i].instance + 1;
        uint8_t destination_index = conversionTable[i].new_index;

        info.old_group_element = conversionTable[i].old_element;
        info.type = (ap_var_type)AP_RangeFinder_Params::var_info[destination_index].type;

        hal.util->snprintf(param_name, sizeof(param_name), "RNGFND%X_%s", param_instance, AP_RangeFinder_Params::var_info[destination_index].name);
        param_name[sizeof(param_name)-1] = '\0';

        AP_Param::convert_old_parameter(&info, 1.0f, 0);
    }

    // force _params[0]._type into storage to flag that conversion has been done
    params[0].type.save(true);
}

/*
  initialise the RangeFinder class. We do detection of attached range
  finders here. For now we won't allow for hot-plugging of
  rangefinders.
 */
void RangeFinder::init(void)
{
    AP_ServoRelayEvents *handler = AP::servorelayevents();
    if (num_instances != 0) {
        // init called a 2nd time?
        return;
    }

    convert_params();
    for (uint8_t i = 0; i < RANGEFINDER_MAX_INSTANCES; i++) {
    	gcs().send_text(MAV_SEVERITY_CRITICAL, "pin off #%d: %d; result: %d", i, i, (handler->do_set_relay(i,1) == 1 ? 1 : 0));
    }
    hal.scheduler->delay(50);
    for (uint8_t i = 0; i < RANGEFINDER_MAX_INSTANCES; i++) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "pin off #%d: %d; result: %d", i, i, (handler->do_set_relay(i,0) == 1 ? 1 : 0));
        params[i].address = 0x30 + i;
        gcs().send_text(MAV_SEVERITY_CRITICAL, "address #%d: %d", i, (int)params[i].address);
    }
    hal.scheduler->delay(50);

    for (uint8_t i=0, serial_instance = 0; i<RANGEFINDER_MAX_INSTANCES; i++) {

    	gcs().send_text(MAV_SEVERITY_CRITICAL, "pin on #%d: %d; result: %d", i, i, (handler->do_set_relay(i,1) == 1 ? 1 : 0));
        hal.scheduler->delay(50);

        // serial_instance will be increased inside detect_instance
        // if a serial driver is loaded for this instance
        detect_instance(i, serial_instance);
        if (drivers[i] != nullptr) {
            // we loaded a driver for this instance, so it must be
            // present (although it may not be healthy)
            num_instances = i+1;
        	gcs().send_text(MAV_SEVERITY_CRITICAL, "ok");
        } else {
        	gcs().send_text(MAV_SEVERITY_CRITICAL, "not");
        }
        // initialise pre-arm check variables
        state[i].pre_arm_check = false;
        state[i].pre_arm_distance_min = 9999;  // initialise to an arbitrary large value
        state[i].pre_arm_distance_max = 0;

        // initialise status
        state[i].status = RangeFinder_NotConnected;
        state[i].range_valid_count = 0;
    }
}

/*
  update RangeFinder state for all instances. This should be called at
  around 10Hz by main loop
 */
void RangeFinder::update(void)
{
    for (uint8_t i=0; i<num_instances; i++) {
        if (drivers[i] != nullptr) {
            if (params[i].type == RangeFinder_TYPE_NONE) {
                // allow user to disable a rangefinder at runtime
                state[i].status = RangeFinder_NotConnected;
                state[i].range_valid_count = 0;
                continue;
            }
            drivers[i]->update();
            drivers[i]->update_pre_arm_check();
        }
    }


	const AP_AHRS &ahrs = AP::ahrs();
    float p = ahrs.pitch;
    float r = ahrs.roll;
    if (cosf(p) < 0.0001f || cosf(r) < 0.0001f)
        return;
    float centered_dist[RANGEFINDER_MAX_INSTANCES];
    float centered_dist_down[RANGEFINDER_MAX_INSTANCES];
    float downward_dist = -1.0f;


    for (uint8_t i = 0; i < num_instances; i++) {
        if (drivers[i] != nullptr && !drivers[i]->has_data()) {
            centered_dist[i] = -2.0f;
            centered_dist_down[i] = -2.0f;
        }
    }

	for (uint8_t i = 0; i < num_instances; i++) {
		if (drivers[i] != nullptr && drivers[i]->has_data()) {
			uint8_t sector = (uint8_t) drivers[i]->orientation();
			AP_Vector3f &s = params[i].pos_offset;
			float s_x = s.get().x * 100.0f;
            float s_y = s.get().y * 100.0f;
            float s_z = s.get().z * 100.0f;

			if (sector == ROTATION_NONE || sector == ROTATION_YAW_180) {
				int signum = sector == ROTATION_NONE ? 1 : -1;
				float t = (s_x * sinf(r) * sinf(p) + s_y * cosf(p) + s_z * cosf(r) * sinf(p)) / cosf(p);
				centered_dist[i] = state[i].distance_cm + t * signum;

				if (fabs(sinf(p)) < 0.0001f) {
				    t = 0.0f;
				} else {
				    t = (-s_x * sinf(r) * cosf(p) + s_y * sinf(p) + s_z * cosf(r) * cosf(p)) / sinf(p);
				}
				centered_dist_down[i] = (state[i].distance_cm + t * signum) * (float)fabs(sinf(p));

//				if (sector == ROTATION_NONE) {
//				    gcs().send_text(MAV_SEVERITY_CRITICAL, "%05.2f  %05.2f  %05.2f",
//				                                (double)((-s_x * sinf(r) * cosf(p))  / sinf(p)),
//                                                (double)((s_y * sinf(p)) / sinf(p)),
//                                                (double)((s_z * cosf(r) * cosf(p))  / sinf(p)));
//				}
			} else if (sector == ROTATION_YAW_90 || sector == ROTATION_YAW_270) {
				int signum = sector == ROTATION_YAW_90 ? 1 : -1;
				float t = (s_x * cosf(r) + s_z * sinf(r)) / cosf(r);
				centered_dist[i] = state[i].distance_cm + t * signum;

                if (fabs(sinf(r)) < 0.0001f) {
                    t = 0.0f;
                } else {
                    t = (-s_x * sinf(r) * cosf(p) + s_y * sinf(p) + s_z * cosf(r) * cosf(p)) / (-sinf(r) * cosf(p));
                }
				centered_dist_down[i] = (state[i].distance_cm + t * signum) * (float)fabs(sinf(r)) * cosf(p);
			} else if (sector == ROTATION_PITCH_90 || sector == ROTATION_PITCH_270) {
				int signum = sector == ROTATION_PITCH_270 ? 1 : -1;
				float t = (s_x * sinf(r) * cosf(p) - s_y * sinf(p) - s_z * cosf(r) * cosf(p)) / (cosf(r) * cosf(p));
				centered_dist[i] = state[i].distance_cm + t * signum;

				if (sector == ROTATION_PITCH_270) {
					downward_dist = centered_dist[i] * (cosf(r) * cosf(p));
//                    gcs().send_text(MAV_SEVERITY_CRITICAL, "%05.2f  %05.2f  %05.2f   %05.2f  %05.2f",
//                            (double)state[i].distance_cm, (double)(t * signum),
//                            (double)(state[i].distance_cm + t * signum), (double)centered_dist[i], (double)downward_dist);
				}
			}
		}
	}


//	{
//        char str[65];     //log for 6 instanses
//        char loc[6];
//        strcpy(str, "cdd: ");
//
//        for (int i = 0; i < num_instances; i++) {
//            uint8_t sector = (uint8_t) drivers[i]->orientation();
//            snprintf(loc, 6, "%d ", (((sector == ROTATION_NONE && p < 0.0f)
//                    || (sector == ROTATION_YAW_180 && p > 0.0f)
//                    || (sector == ROTATION_YAW_90 && r > 0.0f)
//                    || (sector == ROTATION_YAW_270 && r < 0.0f)) &&
//                    (float) fabs(centered_dist_down[i] - downward_dist) < 4.0f) ? 1 : 0);
//            strcat(str, loc);
//        }
//        snprintf(loc, 6, "%03d ", (int) downward_dist);
//        strcat(str, loc);
//        gcs().send_text(MAV_SEVERITY_CRITICAL, str);
//
//    }

//	bool pitch_points_floor = false;
//	bool roll_points_floor = false;
	if (downward_dist >= 0.0f) {            // was processed
		for (uint8_t i = 0; i < num_instances; i++) {
			if (drivers[i] != nullptr && drivers[i]->has_data()) {
				uint8_t sector = (uint8_t) drivers[i]->orientation();
                if ((sector == ROTATION_NONE && p < 0.0f)
                        || (sector == ROTATION_YAW_180 && p > 0.0f)
                        || (sector == ROTATION_YAW_90 && r > 0.0f)
                        || (sector == ROTATION_YAW_270 && r < 0.0f)) {
                    if ((float) fabs(centered_dist_down[i] - downward_dist) < 4.0f) {
                        centered_dist[i] = -3;
                    }
				}

////				if (sector == ROTATION_NONE) {
////				    gcs().send_text(MAV_SEVERITY_CRITICAL, "dp: %3.1f  %3.1f %3.1f %010d",
////				            (double)(centered_dist[i] * (float)fabs(sinf(p))) - (double)downward_dist,
////				         (double)(centered_dist[i] * (float)fabs(sinf(p))), (double)downward_dist, AP_HAL::millis());
////			    }
//				if ((sector == ROTATION_NONE && p < 0.0f) || (sector == ROTATION_YAW_180 && p > 0.0f)) {
//					if ((float)fabs(centered_dist[i] * (float)fabs(sinf(p)) - downward_dist) < 4.0f) {
//						pitch_points_floor = true;
////						gcs().send_text(MAV_SEVERITY_CRITICAL, "dp: %3.1f %010d",
////						        (double)((float)fabs(centered_dist[i] * (float)fabs(sinf(p)) - downward_dist)), AP_HAL::millis());
////						gcs().send_text(MAV_SEVERITY_CRITICAL, "dp: %3.1f %3.1f %010d",
////								(double)(centered_dist[i] * (float)fabs(sinf(p))), (double)downward_dist, AP_HAL::millis());
//					}
//				} else if ((sector == ROTATION_YAW_90 && r > 0.0f) || (sector == ROTATION_YAW_270 && r < 0.0f)) {
//					if ((float)fabs(centered_dist[i] * (float)fabs(sinf(r)) * cosf(p) - downward_dist) < 4.8f) {
//						roll_points_floor = true;
////						gcs().send_text(MAV_SEVERITY_CRITICAL, "dp: %3.1f %3.1f %010d",
////								(double)((float)centered_dist[i] * fabs(sinf(r)) * cosf(p)), (double)downward_dist, AP_HAL::millis());
//					}
//				}
			}
		}
	}

	for (uint8_t i = 0; i < num_instances; i++) {
        if (drivers[i] != nullptr && centered_dist[i] < 0.0f) {
            state[i].distance_cm = params[i].max_distance_cm - 1;
        } else if (drivers[i] != nullptr && drivers[i]->has_data()) {
			uint8_t sector = (uint8_t) drivers[i]->orientation();
            float offseted_dist = centered_dist[i] - params[i].offset * 100.0f;
            if (sector == ROTATION_NONE || sector == ROTATION_YAW_180) {
                state[i].distance_cm = (int)(offseted_dist * cosf(p));
            } else if (sector == ROTATION_YAW_90 || sector == ROTATION_YAW_270) {
                state[i].distance_cm = (int)(offseted_dist * cosf(r));
            } else if (sector == ROTATION_PITCH_90 || sector == ROTATION_PITCH_270) {
                state[i].distance_cm = (int)(offseted_dist * cosf(r) * cosf(p));
            }
            state[i].distance_cm = MAX(state[i].distance_cm, params[i].min_distance_cm + 1);            // trim
			state[i].distance_cm = MIN(state[i].distance_cm, params[i].max_distance_cm - 1);            // trim
		}
	}

    uint32_t now = AP_HAL::millis();
	for (uint8_t i = 0; i < num_instances; i++) {
		if (drivers[i] != nullptr) {
			drivers[i]->add_dist_to_buf(now);
		}
	}

//	{
//	    gcs().send_text(MAV_SEVERITY_CRITICAL, "%6.2f, %6.2f",
//	    		(double)ahrs.pitch, (double)ahrs.roll);
//
//	}


//	{
//		char str[60];     //log for 6 instanses
//		char loc[6];
//		strcpy(str, "dist: ");
//
//		for (int i = 0; i < num_instances; i++) {
//			snprintf(loc, 5, "%03d ", drivers[i]->get_smooth_buf_dist_cm());
////			snprintf(loc, 5, "%03d ", drivers[i]->distance_cm());
//			strcat(str, loc);
//		}
//		gcs().send_text(MAV_SEVERITY_CRITICAL, str);
//
//	}

//	AC_Avoid::get_singleton()->print_log();
}

bool RangeFinder::_add_backend(AP_RangeFinder_Backend *backend)
{
	gcs().send_text(MAV_SEVERITY_CRITICAL, "call _add_backend()");
    if (!backend) {
    	gcs().send_text(MAV_SEVERITY_CRITICAL, "backend == null");
        return false;
    }
    gcs().send_text(MAV_SEVERITY_CRITICAL, "num insts: %d, max: %d", num_instances, RANGEFINDER_MAX_INSTANCES);
    if (num_instances == RANGEFINDER_MAX_INSTANCES) {
        AP_HAL::panic("Too many RANGERS backends");
    }

	gcs().send_text(MAV_SEVERITY_CRITICAL, "drivers++");
    drivers[num_instances++] = backend;
	gcs().send_text(MAV_SEVERITY_CRITICAL, "leave _add_backend()");
    return true;
}

/*
  detect if an instance of a rangefinder is connected. 
 */
void RangeFinder::detect_instance(uint8_t instance, uint8_t& serial_instance)
{
    enum RangeFinder_Type _type = (enum RangeFinder_Type)params[instance].type.get();
    switch (_type) {
    case RangeFinder_TYPE_PLI2C:
    case RangeFinder_TYPE_PLI2CV3:
    case RangeFinder_TYPE_PLI2CV3HP:
        FOREACH_I2C(i) {
            if (_add_backend(AP_RangeFinder_PulsedLightLRF::detect(i, state[instance], params[instance], _type))) {
                break;
            }
        }
        break;
    case RangeFinder_TYPE_MBI2C:
        FOREACH_I2C(i) {
            if (_add_backend(AP_RangeFinder_MaxsonarI2CXL::detect(state[instance], params[instance],
                                                                  hal.i2c_mgr->get_device(i, AP_RANGE_FINDER_MAXSONARI2CXL_DEFAULT_ADDR)))) {
                break;
            }
        }
        break;
    case RangeFinder_TYPE_LWI2C:
        if (params[instance].address) {
#ifdef HAL_RANGEFINDER_LIGHTWARE_I2C_BUS
            _add_backend(AP_RangeFinder_LightWareI2C::detect(state[instance], params[instance],
                hal.i2c_mgr->get_device(HAL_RANGEFINDER_LIGHTWARE_I2C_BUS, params[instance].address)));
#else
            FOREACH_I2C(i) {
                if (_add_backend(AP_RangeFinder_LightWareI2C::detect(state[instance], params[instance],
                                                                     hal.i2c_mgr->get_device(i, params[instance].address)))) {
                    break;
                }
            }
#endif
        }
        break;
    case RangeFinder_TYPE_TRI2C:
        if (params[instance].address) {
            FOREACH_I2C(i) {
                if (_add_backend(AP_RangeFinder_TeraRangerI2C::detect(state[instance], params[instance],
                                                                      hal.i2c_mgr->get_device(i, params[instance].address)))) {
                    break;
                }
            }
        }
        break;
    case RangeFinder_TYPE_VL53L0X:
            FOREACH_I2C(i) {

            	gcs().send_text(MAV_SEVERITY_CRITICAL, "get_device(i, 0x29)->write_register(0x8A, ...)");

            	AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev0x29 = hal.i2c_mgr->get_device(i, 0x29);
            	dev0x29->get_semaphore()->take_blocking();
            	dev0x29->write_register(0x8A, params[instance].address & 0x7F);
            	dev0x29->get_semaphore()->give();


                if (_add_backend(AP_RangeFinder_VL53L0X::detect(state[instance], params[instance],
                                                                 hal.i2c_mgr->get_device(i, params[instance].address)))) {
                    break;
                }
                if (_add_backend(AP_RangeFinder_VL53L1X::detect(state[instance], params[instance],
                                                                hal.i2c_mgr->get_device(i, params[instance].address)))) {
                    break;
                }
            }
        break;
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    case RangeFinder_TYPE_PX4_PWM:
        // to ease moving from PX4 to ChibiOS we'll lie a little about
        // the backend driver...
        if (AP_RangeFinder_PWM::detect()) {
            drivers[instance] = new AP_RangeFinder_PWM(state[instance], params[instance], estimated_terrain_height);
        }
        break;
#endif
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI
    case RangeFinder_TYPE_BBB_PRU:
        if (AP_RangeFinder_BBB_PRU::detect()) {
            drivers[instance] = new AP_RangeFinder_BBB_PRU(state[instance], params[instance]);
        }
        break;
#endif
    case RangeFinder_TYPE_LWSER:
        if (AP_RangeFinder_LightWareSerial::detect(serial_manager, serial_instance)) {
            drivers[instance] = new AP_RangeFinder_LightWareSerial(state[instance], params[instance], serial_manager, serial_instance++);
        }
        break;
    case RangeFinder_TYPE_LEDDARONE:
        if (AP_RangeFinder_LeddarOne::detect(serial_manager, serial_instance)) {
            drivers[instance] = new AP_RangeFinder_LeddarOne(state[instance], params[instance], serial_manager, serial_instance++);
        }
        break;
    case RangeFinder_TYPE_ULANDING:
        if (AP_RangeFinder_uLanding::detect(serial_manager, serial_instance)) {
            drivers[instance] = new AP_RangeFinder_uLanding(state[instance], params[instance], serial_manager, serial_instance++);
        }
        break;
#if (CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP || \
     CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO) && defined(HAVE_LIBIIO)
    case RangeFinder_TYPE_BEBOP:
        if (AP_RangeFinder_Bebop::detect()) {
            drivers[instance] = new AP_RangeFinder_Bebop(state[instance]);
        }
        break;
#endif
    case RangeFinder_TYPE_MAVLink:
        if (AP_RangeFinder_MAVLink::detect()) {
            drivers[instance] = new AP_RangeFinder_MAVLink(state[instance], params[instance]);
        }
        break;
    case RangeFinder_TYPE_MBSER:
        if (AP_RangeFinder_MaxsonarSerialLV::detect(serial_manager, serial_instance)) {
            drivers[instance] = new AP_RangeFinder_MaxsonarSerialLV(state[instance], params[instance], serial_manager, serial_instance++);
        }
        break;
    case RangeFinder_TYPE_ANALOG:
        // note that analog will always come back as present if the pin is valid
        if (AP_RangeFinder_analog::detect(params[instance])) {
            drivers[instance] = new AP_RangeFinder_analog(state[instance], params[instance]);
        }
        break;
    case RangeFinder_TYPE_NMEA:
        if (AP_RangeFinder_NMEA::detect(serial_manager, serial_instance)) {
            drivers[instance] = new AP_RangeFinder_NMEA(state[instance], params[instance], serial_manager, serial_instance++);
        }
        break;
    case RangeFinder_TYPE_WASP:
        if (AP_RangeFinder_Wasp::detect(serial_manager, serial_instance)) {
            drivers[instance] = new AP_RangeFinder_Wasp(state[instance], params[instance], serial_manager, serial_instance++);
        }
        break;
    case RangeFinder_TYPE_BenewakeTF02:
        if (AP_RangeFinder_Benewake::detect(serial_manager, serial_instance)) {
            drivers[instance] = new AP_RangeFinder_Benewake(state[instance], params[instance], serial_manager, serial_instance++, AP_RangeFinder_Benewake::BENEWAKE_TF02);
        }
        break;
    case RangeFinder_TYPE_BenewakeTFmini:
        if (AP_RangeFinder_Benewake::detect(serial_manager, serial_instance)) {
            drivers[instance] = new AP_RangeFinder_Benewake(state[instance], params[instance], serial_manager, serial_instance++, AP_RangeFinder_Benewake::BENEWAKE_TFmini);
        }
        break;
    case RangeFinder_TYPE_PWM:
        if (AP_RangeFinder_PWM::detect()) {
            drivers[instance] = new AP_RangeFinder_PWM(state[instance], params[instance], estimated_terrain_height);
        }
        break;
    default:
        break;
    }

    // if the backend has some local parameters then make those available in the tree
    if (drivers[instance] && state[instance].var_info) {
        backend_var_info[instance] = state[instance].var_info;
        AP_Param::load_object_from_eeprom(drivers[instance], backend_var_info[instance]);
    }
}

AP_RangeFinder_Backend *RangeFinder::get_backend(uint8_t id) const {
    if (id >= num_instances) {
        return nullptr;
    }
    if (drivers[id] != nullptr) {
        if (drivers[id]->type() == RangeFinder_TYPE_NONE) {
            // pretend it isn't here; disabled at runtime?
            return nullptr;
        }
    }
    return drivers[id];
};

RangeFinder::RangeFinder_Status RangeFinder::status_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return RangeFinder_NotConnected;
    }
    return backend->status();
}

void RangeFinder::handle_msg(mavlink_message_t *msg)
{
    uint8_t i;
    for (i=0; i<num_instances; i++) {
        if ((drivers[i] != nullptr) && (params[i].type != RangeFinder_TYPE_NONE)) {
          drivers[i]->handle_msg(msg);
        }
    }
}

// return true if we have a range finder with the specified orientation
bool RangeFinder::has_orientation(enum Rotation orientation) const
{
    return (find_instance(orientation) != nullptr);
}

// find first range finder instance with the specified orientation
AP_RangeFinder_Backend *RangeFinder::find_instance(enum Rotation orientation) const
{
    for (uint8_t i=0; i<num_instances; i++) {
        AP_RangeFinder_Backend *backend = get_backend(i);
        if (backend == nullptr) {
            continue;
        }
        if (backend->orientation() != orientation) {
            continue;
        }
        return backend;
    }
    return nullptr;
}

uint16_t RangeFinder::distance_cm_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;
    }
    return backend->distance_cm();
}

uint16_t RangeFinder::voltage_mv_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;
    }
    return backend->voltage_mv();
}

int16_t RangeFinder::max_distance_cm_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;
    }
    return backend->max_distance_cm();
}

int16_t RangeFinder::min_distance_cm_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;
    }
    return backend->min_distance_cm();
}

int16_t RangeFinder::ground_clearance_cm_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;
    }
    return backend->ground_clearance_cm();
}

bool RangeFinder::has_data_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return false;
    }
    return backend->has_data();
}

uint8_t RangeFinder::range_valid_count_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;
    }
    return backend->range_valid_count();
}

/*
  returns true if pre-arm checks have passed for all range finders
  these checks involve the user lifting or rotating the vehicle so that sensor readings between
  the min and 2m can be captured
 */
bool RangeFinder::pre_arm_check() const
{
    for (uint8_t i=0; i<num_instances; i++) {
        // if driver is valid but pre_arm_check is false, return false
        if ((drivers[i] != nullptr) && (params[i].type != RangeFinder_TYPE_NONE) && !state[i].pre_arm_check) {
            return false;
        }
    }
    return true;
}

const Vector3f &RangeFinder::get_pos_offset_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return pos_offset_zero;
    }
    return backend->get_pos_offset();
}

uint32_t RangeFinder::last_reading_ms(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;
    }
    return backend->last_reading_ms();
}

MAV_DISTANCE_SENSOR RangeFinder::get_mav_distance_sensor_type_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return MAV_DISTANCE_SENSOR_UNKNOWN;
    }
    return backend->get_mav_distance_sensor_type();
}

RangeFinder *RangeFinder::_singleton;

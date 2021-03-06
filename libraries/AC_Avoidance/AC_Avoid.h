#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>     // AHRS library
#include <AC_AttitudeControl/AC_AttitudeControl.h> // Attitude controller library for sqrt controller
#include <AC_Fence/AC_Fence.h>         // Failsafe fence library
#include <AP_Proximity/AP_Proximity.h>
#include <AP_Beacon/AP_Beacon.h>

#define AC_AVOID_ACCEL_CMSS_MAX         100.0f  // maximum acceleration/deceleration in cm/s/s used to avoid hitting fence

// bit masks for enabled fence types.
#define AC_AVOID_DISABLED               0       // avoidance disabled
#define AC_AVOID_STOP_AT_FENCE          1       // stop at fence
#define AC_AVOID_USE_PROXIMITY_SENSOR   2       // stop based on proximity sensor output
#define AC_AVOID_STOP_AT_BEACON_FENCE   4       // stop based on beacon perimeter
#define AC_AVOID_DEFAULT                (AC_AVOID_STOP_AT_FENCE | AC_AVOID_USE_PROXIMITY_SENSOR)

// definitions for non-GPS avoidance
#define AC_AVOID_NONGPS_DIST_MAX_DEFAULT    5.0f    // objects over 5m away are ignored (default value for DIST_MAX parameter)
#define AC_AVOID_ANGLE_MAX_PERCENT          0.88f   // object avoidance max lean angle as a percentage (expressed in 0 ~ 1 range) of total vehicle max lean angle

#define AC_AVOID_HOR_PID_P_DEFAULT          4000.0f
#define AC_AVOID_HOR_PID_D_DEFAULT          500.0f
#define AC_AVOID_VER_PID_P_DEFAULT          0.1f
#define AC_AVOID_VER_PID_D_DEFAULT          0.0002f
#define AC_AVOID_VER_PID_H_DEFAULT          0.7f
#define AC_AVOID_VER_CHAN_DEFAULT           5

/*
 * This class prevents the vehicle from leaving a polygon fence in
 * 2 dimensions by limiting velocity (adjust_velocity).
 */
class AC_Avoid {
public:
    AC_Avoid(const AP_AHRS& ahrs, const AC_Fence& fence, const AP_Proximity& proximity, const AP_Beacon* beacon = nullptr);

    /* Do not allow copies */
    AC_Avoid(const AC_Avoid &other) = delete;
    AC_Avoid &operator=(const AC_Avoid&) = delete;

    // get singleton instance
    static AC_Avoid *get_singleton() {
        return _singleton;
    }

    /*
     * Adjusts the desired velocity so that the vehicle can stop
     * before the fence/object.
     * Note: Vector3f version is for convenience and only adjusts x and y axis
     */
    void adjust_velocity(float kP, float accel_cmss, Vector2f &desired_vel_cms, float dt);
    void adjust_velocity(float kP, float accel_cmss, Vector3f &desired_vel_cms, float dt);

    // adjust desired horizontal speed so that the vehicle stops before the fence or object
    // accel (maximum acceleration/deceleration) is in m/s/s
    // heading is in radians
    // speed is in m/s
    // kP should be zero for linear response, non-zero for non-linear response
    // dt is the time since the last call in seconds
    void adjust_speed(float kP, float accel, float heading, float &speed, float dt);

    // adjust vertical climb rate so vehicle does not break the vertical fence
    void adjust_velocity_z(float kP, float accel_cmss, float& climb_rate_cms, float dt);

    // adjust roll-pitch to push vehicle away from objects
    // roll and pitch value are in centi-degrees
    // angle_max is the user defined maximum lean angle for the vehicle in centi-degrees
    void adjust_roll_pitch(float &roll, float &pitch, float angle_max);
    void adjust_throttle(float &throttle_scaled);

    // enable/disable proximity based avoidance
    void proximity_avoidance_enable(bool on_off) { _proximity_enabled = on_off; }
    bool proximity_avoidance_enabled() { return _proximity_enabled; }

    // helper functions

    // Limits the component of desired_vel_cms in the direction of the unit vector
    // limit_direction to be at most the maximum speed permitted by the limit_distance_cm.
    // uses velocity adjustment idea from Randy's second email on this thread:
    //   https://groups.google.com/forum/#!searchin/drones-discuss/obstacle/drones-discuss/QwUXz__WuqY/qo3G8iTLSJAJ
    void limit_velocity(float kP, float accel_cmss, Vector2f &desired_vel_cms, const Vector2f& limit_direction, float limit_distance_cm, float dt) const;

     // compute the speed such that the stopping distance of the vehicle will
     // be exactly the input distance.
     // kP should be non-zero for Copter which has a non-linear response
    float get_max_speed(float kP, float accel_cmss, float distance_cm, float dt) const;




    void print_log();




    static const struct AP_Param::GroupInfo var_info[];

private:
    // behaviour types (see BEHAVE parameter)
    enum BehaviourType {
        BEHAVIOR_SLIDE = 0,
        BEHAVIOR_STOP = 1
    };

    /*
     * Adjusts the desired velocity for the circular fence.
     */
    void adjust_velocity_circle_fence(float kP, float accel_cmss, Vector2f &desired_vel_cms, float dt);

    /*
     * Adjusts the desired velocity for the polygon fence.
     */
    void adjust_velocity_polygon_fence(float kP, float accel_cmss, Vector2f &desired_vel_cms, float dt);

    /*
     * Adjusts the desired velocity for the beacon fence.
     */
    void adjust_velocity_beacon_fence(float kP, float accel_cmss, Vector2f &desired_vel_cms, float dt);

    /*
     * Adjusts the desired velocity based on output from the proximity sensor
     */
    void adjust_velocity_proximity(float kP, float accel_cmss, Vector2f &desired_vel_cms, float dt);

    /*
     * Adjusts the desired velocity given an array of boundary points
     *   earth_frame should be true if boundary is in earth-frame, false for body-frame
     *   margin is the distance (in meters) that the vehicle should stop short of the polygon
     */
    void adjust_velocity_polygon(float kP, float accel_cmss, Vector2f &desired_vel_cms, const Vector2f* boundary, uint16_t num_points, bool earth_frame, float margin, float dt);

    /*
     * Computes distance required to stop, given current speed.
     */
    float get_stopping_distance(float kP, float accel_cmss, float speed_cms) const;

    /*
     * methods for avoidance in non-GPS flight modes
     */

    // convert distance (in meters) to a lean percentage (in 0~1 range) for use in manual flight modes
    float distance_to_lean_pct(float dist_m);

    void get_proximity_distance_der(float &roll_dist_der, float &pitch_dist_der);
    // returns the maximum positive and negative roll and pitch percentages (in -1 ~ +1 range) based on the proximity sensor
    void get_proximity_roll_pitch_pct(float &roll_positive, float &roll_negative, float &pitch_positive, float &pitch_negative);

    // external references
    const AP_AHRS& _ahrs;
    const AC_Fence& _fence;
    const AP_Proximity& _proximity;
    const AP_Beacon* _beacon;

    // parameters
    AP_Int8 _enabled;
    AP_Int16 _angle_max;        // maximum lean angle to avoid obstacles (only used in non-GPS flight modes)
    AP_Float _dist_max;         // distance (in meters) from object at which obstacle avoidance will begin in non-GPS modes
    AP_Float _margin;           // vehicle will attempt to stay this distance (in meters) from objects while in GPS modes
    AP_Int8 _behavior;          // avoidance behaviour (slide or stop)
    AP_Float _hor_pid_p;        // Maximum angle for P-component in horizontal PID (via RNGFND) witch would turn out if there was not ANGLE_MAX limit
    AP_Float _hor_pid_d;        // D-component in horizontal PID (via RNGFND)
    AP_Float _ver_pid_p;        // Target climbing velocity in vertical PID (via RNGFND)
    AP_Float _ver_pid_d;        // D-component in vertical PID (via RNGFND)
    AP_Float _ver_pid_h;        // Target altitude in vertical PID (via RNGFND)
    AP_Int8 _ver_chan;          // Target altitude in vertical PID to avoid obstacles (via rangefinder)

    bool _proximity_enabled = true; // true if proximity sensor based avoidance is enabled (used to allow pilot to enable/disable)

    static AC_Avoid *_singleton;
};

namespace AP {
    AC_Avoid *ac_avoid();
};

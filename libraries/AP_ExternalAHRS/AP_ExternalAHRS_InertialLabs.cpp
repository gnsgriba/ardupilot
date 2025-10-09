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
/*
  support for serial connected InertialLabs INS
 */

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED

#include "AP_ExternalAHRS_InertialLabs.h"
#include "InertialLabs_command.h"
#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Common/Bitmask.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

extern const AP_HAL::HAL &hal;

AP_ExternalAHRS_InertialLabs::AP_ExternalAHRS_InertialLabs(AP_ExternalAHRS *_frontend,
                                                           AP_ExternalAHRS::state_t &_state) :
    AP_ExternalAHRS_backend(_frontend, _state)
{
    // don't offer IMU by default, at 200Hz it is too slow for many aircraft
    set_default_sensors(uint16_t(AP_ExternalAHRS::AvailableSensor::GPS) |
                        uint16_t(AP_ExternalAHRS::AvailableSensor::BARO) |
                        uint16_t(AP_ExternalAHRS::AvailableSensor::COMPASS));

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_InertialLabs::update_thread, void), "ILabs", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        AP_HAL::panic("InertialLabs Failed to start ExternalAHRS update thread");
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "InertialLabs ExternalAHRS initialised");
}

int8_t AP_ExternalAHRS_InertialLabs::get_port() const
{
    return sensor.get_port();
};

bool AP_ExternalAHRS_InertialLabs::healthy() const
{
    WITH_SEMAPHORE(state.sem);
    return AP_HAL::millis() - last_sensor_data.att_ms < 100;
}

bool AP_ExternalAHRS_InertialLabs::initialised() const
{
    return sensor.is_initialized();
}

bool AP_ExternalAHRS_InertialLabs::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    if (!sensor.is_initialized()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "InertialLabs setup failed");
        return false;
    }
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "InertialLabs unhealthy");
        return false;
    }
    WITH_SEMAPHORE(state.sem);
    uint32_t now = AP_HAL::millis();
    if (now - last_sensor_data.att_ms > 10 ||
        now - last_sensor_data.pos_ms > 10 ||
        now - last_sensor_data.vel_ms > 10) {
        hal.util->snprintf(failure_msg, failure_msg_len, "InertialLabs not up to date");
        return false;
    }
    return true;
}

void AP_ExternalAHRS_InertialLabs::get_filter_status(nav_filter_status &status) const
{
    // We don't know the meaning of the status bits yet, so assume all OK if we have GPS lock
    using InertialLabs::USW;

    WITH_SEMAPHORE(state.sem);

    const InertialLabs::SensorsData &sensors_data = sensor.get_sensors_data();

    uint32_t now = AP_HAL::millis();
    const uint32_t dt_limit = 200;
    const uint32_t dt_limit_gps = 500;
    memset(&status, 0, sizeof(status));

    const bool init_ok = (sensors_data.ins.unit_status & (USW::INITIAL_ALIGNMENT_FAIL|USW::OPERATION_FAIL)) == 0;

    status.flags.initalized = init_ok;

    status.flags.attitude = init_ok && (now - last_sensor_data.att_ms < dt_limit);
    status.flags.vert_vel = init_ok && (now - last_sensor_data.vel_ms < dt_limit);
    status.flags.vert_pos = init_ok && (now - last_sensor_data.pos_ms < dt_limit);
    status.flags.horiz_vel = status.flags.vert_vel;
    status.flags.horiz_pos_abs = status.flags.vert_pos;
    status.flags.horiz_pos_rel = status.flags.vert_pos;
    status.flags.pred_horiz_pos_abs = status.flags.vert_pos;

    status.flags.using_gps = (now - last_sensor_data.gps_ms < dt_limit_gps);
    status.flags.gps_quality_good = (now - last_sensor_data.gps_ms < dt_limit_gps);

    status.flags.rejecting_airspeed = false;
}

bool AP_ExternalAHRS_InertialLabs::get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const
{
    // TODO: implement this variances calculation
    velVar = 0;
    posVar = 0;
    hgtVar = 0;
    tasVar = 0;
    return true;
}

void AP_ExternalAHRS_InertialLabs::write_bytes(const char *bytes, uint8_t len)
{
    sensor.write_bytes(bytes, len);
}

void AP_ExternalAHRS_InertialLabs::handle_command(ExternalAHRS_command command, const ExternalAHRS_command_data &data)
{
    switch (command) {
        case ExternalAHRS_command::START_UDD:
            write_bytes(InertialLabs::Command::START_UDD,
                        sizeof(InertialLabs::Command::START_UDD) - 1);
            break;
        case ExternalAHRS_command::STOP:
            write_bytes(InertialLabs::Command::STOP,
                        sizeof(InertialLabs::Command::STOP) - 1);
            break;
        case ExternalAHRS_command::ENABLE_GNSS:
            write_bytes(InertialLabs::Command::ENABLE_GNSS,
                        sizeof(InertialLabs::Command::ENABLE_GNSS) - 1);
            break;
        case ExternalAHRS_command::DISABLE_GNSS:
            write_bytes(InertialLabs::Command::DISABLE_GNSS,
                        sizeof(InertialLabs::Command::DISABLE_GNSS) - 1);
            break;
        case ExternalAHRS_command::START_VG3D_CALIBRATION_IN_FLIGHT:
            write_bytes(InertialLabs::Command::START_VG3DCLB_FLIGHT,
                        sizeof(InertialLabs::Command::START_VG3DCLB_FLIGHT) - 1);
            break;
        case ExternalAHRS_command::STOP_VG3D_CALIBRATION_IN_FLIGHT:
            write_bytes(InertialLabs::Command::STOP_VG3DCLB_FLIGHT,
                        sizeof(InertialLabs::Command::STOP_VG3DCLB_FLIGHT) - 1);
            break;
        case ExternalAHRS_command::AIDING_DATA_EXTERNAL_POSITION:
        case ExternalAHRS_command::AIDING_DATA_EXTERNAL_HORIZONTAL_POSITION:
        case ExternalAHRS_command::AIDING_DATA_EXTERNAL_ALTITUDE:
        case ExternalAHRS_command::AIDING_DATA_WIND:
        case ExternalAHRS_command::AIDING_DATA_AMBIENT_AIR:
        case ExternalAHRS_command::AIDING_DATA_EXTERNAL_HEADING:
        {
            InertialLabs::Data_context context;
            InertialLabs::fill_command_pyload(context, command, data);
            InertialLabs::fill_transport_protocol_data(context);
            write_bytes(reinterpret_cast<const char *>(context.data), context.length);
            break;
        }

        default:
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ILAB: Invalid command for handling");
    }
}

bool AP_ExternalAHRS_InertialLabs::get_wind_estimation(Vector3f &wind)
{
    const InertialLabs::SensorsData &sensors_data = sensor.get_sensors_data();
    wind = sensors_data.ins.wind_speed;
    return true;
}

void AP_ExternalAHRS_InertialLabs::send_eahrs_status_flag(GCS_MAVLINK &link) const
{
    const InertialLabs::SensorsData &sensors_data = sensor.get_sensors_data();

    const mavlink_eahrs_status_info_t package{sensors_data.ins.unit_status,
                                              sensors_data.ins.unit_status2,
                                              sensors_data.ins.air_data_status,
                                              (uint16_t)(sensors_data.gps.fix_type ? sensors_data.gps.fix_type-1 : 0), //< Send ILabs AHRS output as is. Without inc
                                              sensors_data.gps.spoof_status};
    mavlink_msg_eahrs_status_info_send_struct(link.get_chan(), &package);
}

void AP_ExternalAHRS_InertialLabs::update()
{
    handle_full_circle();
}

bool AP_ExternalAHRS_InertialLabs::handle_full_circle()
{
    if (!sensor.is_initialized()) {
        return false;
    }

    WITH_SEMAPHORE(state.sem);

    if (!sensor.update_data()) {
        return false;
    }

    handle_sensor_data();
    send_data_to_sensor();
    write_logs();
    send_GCS_messages();
    return true;
}

void AP_ExternalAHRS_InertialLabs::update_thread()
{
    if (!sensor.init()) {
        AP_HAL::panic("InertialLabs Failed to initialize sensor");
    }

    while (true) {
        handle_full_circle();
        hal.scheduler->delay_microseconds(250);
    }
}

void AP_ExternalAHRS_InertialLabs::handle_sensor_data()
{
    using InertialLabs::ADU;
    using InertialLabs::InsSolution;
    using InertialLabs::NewGPSData;
    using InertialLabs::USW;
    using InertialLabs::USW2;

    const InertialLabs::SensorsData &sensors_data = sensor.get_sensors_data();

    const bool filter_ok = (sensors_data.ins.unit_status & USW::INITIAL_ALIGNMENT_FAIL) == 0 &&
                           (sensors_data.ins.ins_sol_status != InsSolution::INVALID);

    if (filter_ok) {
        // use IL INS attitude data in the ArduPilot algorithm instead of EKF3 or DCM
        state.quat.from_euler(static_cast<float>(radians(sensors_data.ins.roll)),
                              static_cast<float>(radians(sensors_data.ins.pitch)),
                              static_cast<float>(radians(sensors_data.ins.yaw)));
        state.have_quaternion = true;
        if (last_sensor_data.att_ms == 0) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "InertialLabs: got link");
        }

        last_sensor_data.att_ms = sensors_data.package_timestamp_ms;
    }

    if (filter_ok && (sensors_data.ins.unit_status & (USW::GYRO_FAIL|USW::ACCEL_FAIL)) == 0) {
        // use IL INS IMU outputs in the ArduPilot algorithm
        state.accel = sensors_data.accel;
        state.gyro = sensors_data.gyro;
        ins_data.accel = sensors_data.accel;
        ins_data.gyro = sensors_data.gyro;
        ins_data.temperature = sensors_data.temperature;
        AP::ins().handle_external(ins_data);
    }

    const bool hasNewGpsData = (sensors_data.gps.new_data & (NewGPSData::NEW_GNSS_POSITION|NewGPSData::NEW_GNSS_VELOCITY)) != 0; // true if received new GNSS position or velocity

    if (filter_ok) {
        // use IL INS navigation solution instead of EKF3 or DCM
        state.location.lat = sensors_data.ins.latitude;
        state.location.lng = sensors_data.ins.longitude;
        state.location.alt = sensors_data.ins.altitude;
        state.velocity = sensors_data.ins.velocity;
        state.have_velocity = true;
        state.have_location = true;
        state.last_location_update_us = AP_HAL::micros();

        last_sensor_data.vel_ms = sensors_data.package_timestamp_ms;
        last_sensor_data.pos_ms = sensors_data.package_timestamp_ms;

        gps_data.ins_lat_accuracy = static_cast<uint32_t>(sensors_data.ins.ins_accuracy.lat);
        gps_data.ins_lng_accuracy = static_cast<uint32_t>(sensors_data.ins.ins_accuracy.lon);
        gps_data.ins_alt_accuracy = static_cast<uint32_t>(sensors_data.ins.ins_accuracy.alt);

        if (hasNewGpsData) {
            // use IL INS navigation solution instead of GNSS solution
            gps_data.ms_tow = sensors_data.ins.ms_tow;
            gps_data.gps_week = sensors_data.gps.gps_week;
            gps_data.latitude = sensors_data.ins.latitude;
            gps_data.longitude = sensors_data.ins.longitude;
            gps_data.msl_altitude = sensors_data.ins.altitude;
            gps_data.ned_vel_north = sensors_data.ins.velocity.x;
            gps_data.ned_vel_east = sensors_data.ins.velocity.y;
            gps_data.ned_vel_down = sensors_data.ins.velocity.z;

            const bool gps_sol_trick = option_is_set(AP_ExternalAHRS::OPTIONS::ILAB_DISABLE_GPS_TRICK);
            const bool gps_solution = ((sensors_data.ins.unit_status2 & USW2::GNSS_FUSION_OFF) == 0) &&
                                      (sensors_data.gps.gnss_sol_status == InsSolution::GOOD) &&
                                      (sensors_data.gps.fix_type == 2);
            if (gps_sol_trick || gps_solution) { // use valid GNSS data as is
                gps_data.fix_type = AP_GPS_FixType::FIX_3D;
                gps_data.satellites_in_view = sensors_data.gps.full_sat_info.SolnSVs;
                gps_data.hdop = static_cast<float>(sensors_data.gps.dop.hdop)*0.1f;
                gps_data.vdop = static_cast<float>(sensors_data.gps.dop.vdop)*0.1f;
            } else { // set fixed values to continue normal flight in GNSS-denied environments
                gps_data.fix_type = AP_GPS_FixType::FIX_3D;
                gps_data.satellites_in_view = 77;
                gps_data.hdop = 90.0f; // 0.9
                gps_data.vdop = 90.0f; // 0.9
            }
            gps_data.latitude_raw = sensors_data.gps.latitude;
            gps_data.longitude_raw = sensors_data.gps.longitude;
            gps_data.altitude_raw = sensors_data.gps.altitude;
            gps_data.track_over_ground_raw = static_cast<int32_t>(sensors_data.gps.track_over_ground*100.0f);
            gps_data.gps_raw_status = sensors_data.gps.gnss_sol_status;

            gps_data.latitude_raw = sensors_data.gps.latitude;
            gps_data.longitude_raw = sensors_data.gps.longitude;
            gps_data.altitude_raw = sensors_data.gps.altitude;
            gps_data.track_over_ground_raw = static_cast<int32_t>(sensors_data.gps.track_over_ground);
            gps_data.gps_raw_status = sensors_data.gps.gnss_sol_status;

            uint8_t instance{0};
            if (AP::gps().get_first_external_instance(instance)) {
                AP::gps().handle_external(gps_data, instance);
            }
            if (gps_data.satellites_in_view > 3) {
                if (last_sensor_data.gps_ms == 0) {
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "InertialLabs: got GPS lock");
                    if (!state.have_origin) {
                        state.origin = Location{
                            gps_data.latitude,
                            gps_data.longitude,
                            gps_data.msl_altitude,
                            Location::AltFrame::ABSOLUTE};
                        state.have_origin = true;
                    }
                }
                last_sensor_data.gps_ms = sensors_data.package_timestamp_ms;
            }
        }
    }

#if AP_BARO_EXTERNALAHRS_ENABLED
    if ((sensors_data.ins.unit_status2 & USW2::ADU_BARO_FAIL) == 0) {
        // use IL INS barometer output in the ArduPilot algorithm
        baro_data.pressure_pa = sensors_data.pressure;
        baro_data.temperature = sensors_data.temperature;
        AP::baro().handle_external(baro_data);
    }
#endif

#if AP_COMPASS_EXTERNALAHRS_ENABLED
    if ((sensors_data.ins.unit_status & USW::MAG_FAIL) == 0) {
        // use IL INS magnetometer outputs in the ArduPilot algorithm
        mag_data.field = sensors_data.mag;
        AP::compass().handle_external(mag_data);
    }
#endif

#if AP_AIRSPEED_EXTERNAL_ENABLED && (APM_BUILD_COPTER_OR_HELI || APM_BUILD_TYPE(APM_BUILD_ArduPlane))
    // only on plane and copter as others do not link AP_Airspeed
    if ((sensors_data.ins.unit_status2 & USW2::ADU_DIFF_PRESS_FAIL) == 0) {
        airspeed_data.differential_pressure = sensors_data.diff_press;
        airspeed_data.temperature = sensors_data.temperature;
        airspeed_data.airspeed = sensors_data.ins.true_airspeed;
        auto *arsp = AP::airspeed();
        if (arsp != nullptr) {
            if (option_is_set(AP_ExternalAHRS::OPTIONS::ILAB_USE_AIRSPEED)) {
                // use IL INS calculated true airspeed
                bool airspeed_enabled = false;
                if (filter_ok && (sensors_data.ins.air_data_status & ADU::AIRSPEED_FAIL) == 0) {
                    airspeed_enabled = true;
                }
                arsp->set_external_airspeed_enabled(airspeed_enabled);
            }
            arsp->handle_external(airspeed_data);
        }
    }
#endif // AP_AIRSPEED_EXTERNAL_ENABLED
}

void AP_ExternalAHRS_InertialLabs::write_logs()
{
    using InertialLabs::NewAidingData;

    const InertialLabs::SensorsData &sensors_data = sensor.get_sensors_data();

#if HAL_LOGGING_ENABLED
    uint64_t now_us = static_cast<uint64_t>(sensors_data.package_timestamp_ms) * 1000; // TODO: Is it Ok precision?

    // @LoggerMessage: ILB1
    // @Description: InertialLabs IMU and Mag data
    // @Field: TimeUS: Time since system startup
    // @Field: IMS: GPS INS time (round)
    // @Field: GyrX: Gyro X
    // @Field: GyrY: Gyro Y
    // @Field: GyrZ: Gyro z
    // @Field: AccX: Accelerometer X
    // @Field: AccY: Accelerometer Y
    // @Field: AccZ: Accelerometer Z
    // @Field: MagX: Magnetometer X
    // @Field: MagY: Magnetometer Y
    // @Field: MagZ: Magnetometer Z

    AP::logger().WriteStreaming("ILB1", "TimeUS,IMS,GyrX,GyrY,GyrZ,AccX,AccY,AccZ,MagX,MagY,MagZ",
                                "s-EEEoooGGG",
                                "F----------",
                                "QIfffffffff",
                                now_us, sensors_data.ins.ms_tow,
                                sensors_data.gyro.x, sensors_data.gyro.y, sensors_data.gyro.z,
                                sensors_data.accel.x, sensors_data.accel.y, sensors_data.accel.z,
                                sensors_data.mag.x, sensors_data.mag.y, sensors_data.mag.z);

    // @LoggerMessage: ILBX
    // @Description: InertialLabs sensors bias data
    // @Field: TimeUS: Time since system startup
    // @Field: IMS: GPS INS time (round)
    // @Field: GyrX: Gyro bias X
    // @Field: GyrY: Gyro bias Y
    // @Field: GyrZ: Gyro bias Z
    // @Field: AccX: Accel bias X
    // @Field: AccY: Accel bias Y
    // @Field: AccZ: Accel bias Z

    AP::logger().WriteStreaming("ILBX", "TimeUS,IMS,GyrX,GyrY,GyrZ,AccX,AccY,AccZ",
                                "s-kkk---",
                                "F-------",
                                "QIffffff",
                                now_us, sensors_data.ins.ms_tow,
                                static_cast<float>(sensors_data.ins.sensor_bias.gyroY)*2.0f*1.0e-4f,
                                static_cast<float>(sensors_data.ins.sensor_bias.gyroX)*2.0f*1.0e-4f,
                                static_cast<float>(sensors_data.ins.sensor_bias.gyroZ)*2.0f*1.0e-4f*(-1.0f),
                                static_cast<float>(sensors_data.ins.sensor_bias.accY)*2.0f*1.0e-5f,
                                static_cast<float>(sensors_data.ins.sensor_bias.accX)*2.0f*1.0e-5f,
                                static_cast<float>(sensors_data.ins.sensor_bias.accZ)*2.0f*1.0e-5f*(-1.0f));

    // @LoggerMessage: ILB2
    // @Description: InertialLabs ADC data
    // @Field: TimeUS: Time since system startup
    // @Field: IMS: GPS INS time (round)
    // @Field: Press: Static pressure
    // @Field: Diff: Differential pressure
    // @Field: Temp: Temperature
    // @Field: Alt: Baro altitude
    // @Field: TAS: true airspeed
    // @Field: CAS: calibrated airspeed
    // @Field: VWN: Wind velocity north
    // @Field: VWE: Wind velocity east
    // @Field: ArspSF: The scale factor (SF) for measured air speed

    AP::logger().WriteStreaming("ILB2", "TimeUS,IMS,Press,Diff,Temp,Alt,TAS,CAS,VWN,VWE,ArspSF",
                                "s-PPOmnnnn-",
                                "F----------",
                                "QIfffffffff",
                                now_us, sensors_data.ins.ms_tow,
                                sensors_data.pressure, sensors_data.diff_press, sensors_data.temperature,
                                sensors_data.ins.baro_alt, sensors_data.ins.true_airspeed, sensors_data.ins.calibrated_airspeed,
                                sensors_data.ins.wind_speed.x, sensors_data.ins.wind_speed.y, sensors_data.ins.airspeed_sf);

    // @LoggerMessage: ILB3
    // @Description: InertialLabs INS data
    // @Field: TimeUS: Time since system startup
    // @Field: IMS: GPS INS time (round)
    // @Field: Roll: euler roll
    // @Field: Pitch: euler pitch
    // @Field: Yaw: euler yaw
    // @Field: VN: velocity north
    // @Field: VE: velocity east
    // @Field: VD: velocity down
    // @Field: Lat: latitude
    // @Field: Lng: longitude
    // @Field: Alt: altitude

    AP::logger().WriteStreaming("ILB3", "TimeUS,IMS,Roll,Pitch,Yaw,VN,VE,VD,Lat,Lng,Alt",
                                "s-dddnnnDUm",
                                "F----------",
                                "QIffffffddf",
                                now_us, sensors_data.ins.ms_tow,
                                sensors_data.ins.roll, sensors_data.ins.pitch, sensors_data.ins.yaw,
                                sensors_data.ins.velocity.x, sensors_data.ins.velocity.y, sensors_data.ins.velocity.z,
                                static_cast<double>(sensors_data.ins.latitude)*1.0e-7,
                                static_cast<double>(sensors_data.ins.longitude)*1.0e-7,
                                static_cast<float>(sensors_data.ins.altitude)*0.01f);

    // @LoggerMessage: ILB9
    // @Description: InertialLabs service data
    // @Field: TimeUS: Time since system startup
    // @Field: IMS: GPS INS time (round)
    // @Field: USW: Unit Status Word
    // @Field: USW2: Unit Status Word 2
    // @Field: ADU: Air Data Unit status
    // @Field: ISS: INS Navigation (Solution) Status
    // @Field: NAD1: New Aiding Data
    // @Field: NAD2: New Aiding Data 2
    // @Field: Vdc: Supply voltage

    AP::logger().WriteStreaming("ILB9", "TimeUS,IMS,USW,USW2,ADU,ISS,NAD1,NAD2,Vdc",
                                "s-------v",
                                "F--------",
                                "QIHHHBHHf",
                                now_us, sensors_data.ins.ms_tow, sensors_data.ins.unit_status, sensors_data.ins.unit_status2,
                                sensors_data.ins.air_data_status, sensors_data.ins.ins_sol_status,
                                sensors_data.ext.new_aiding_data, sensors_data.ext.new_aiding_data2, sensors_data.supply_voltage);

    if (sensors_data.gps.new_data != 0) {
        // @LoggerMessage: ILB4
        // @Description: InertialLabs GPS data1
        // @Field: TimeUS: Time since system startup
        // @Field: IMS: GPS INS time (round)
        // @Field: GMS: GNSS Position timestamp
        // @Field: GWk: GPS Week
        // @Field: FType: fix type
        // @Field: NewGPS: Indicator of new update of GPS data
        // @Field: Lat: GNSS Latitude
        // @Field: Lng: GNSS Longitude
        // @Field: Alt: GNSS Altitude
        // @Field: GCrs: GNSS Track over ground
        // @Field: Spd: GNSS Horizontal speed
        // @Field: VZ: GNSS Vertical speed

        AP::logger().WriteStreaming("ILB4", "TimeUS,IMS,GMS,GWk,FType,NewGPS,Lat,Lng,Alt,GCrs,Spd,VZ",
                                    "s-----DUmhnn",
                                    "F-----------",
                                    "QIIHBBddffff",
                                    now_us, sensors_data.ins.ms_tow, sensors_data.gps.ms_tow, sensors_data.gps.gps_week,
                                    sensors_data.gps.fix_type, sensors_data.gps.new_data,
                                    static_cast<double>(sensors_data.gps.latitude)*1.0e-7,
                                    static_cast<double>(sensors_data.gps.longitude)*1.0e-7,
                                    static_cast<float>(sensors_data.gps.altitude)*0.01f,
                                    sensors_data.gps.track_over_ground, sensors_data.gps.hor_speed, sensors_data.gps.ver_speed);

        // @LoggerMessage: ILB5
        // @Description: InertialLabs GPS data2
        // @Field: TimeUS: Time since system startup
        // @Field: IMS: GPS INS time (round)
        // @Field: GSS: GNSS spoofing status
        // @Field: GJS: GNSS jamming status
        // @Field: VL: GNSS Velocity latency
        // @Field: SolS: GNSS Solution status
        // @Field: PVT: GNSS Position or Velocity type
        // @Field: GDOP: GNSS GDOP
        // @Field: PDOP: GNSS PDOP
        // @Field: HDOP: GNSS HDOP
        // @Field: VDOP: GNSS VDOP
        // @Field: TDOP: GNSS TDOP

        AP::logger().WriteStreaming("ILB5", "TimeUS,IMS,GSS,GJS,VL,SolS,PVT,GDOP,PDOP,HDOP,VDOP,TDOP",
                                    "s-----------",
                                    "F-----------",
                                    "QIBBHBBfffff",
                                    now_us, sensors_data.ins.ms_tow,
                                    sensors_data.gps.spoof_status, sensors_data.gps.jam_status, sensors_data.gps.vel_latency,
                                    sensors_data.gps.gnss_sol_status, sensors_data.gps.gnss_pos_vel_type,
                                    static_cast<float>(sensors_data.gps.dop.gdop)*1.0e-3f,
                                    static_cast<float>(sensors_data.gps.dop.pdop)*1.0e-3f,
                                    static_cast<float>(sensors_data.gps.dop.hdop)*1.0e-3f,
                                    static_cast<float>(sensors_data.gps.dop.vdop)*1.0e-3f,
                                    static_cast<float>(sensors_data.gps.dop.tdop)*1.0e-3f);

        // @LoggerMessage: ILB6
        // @Description: InertialLabs GPS data3
        // @Field: TimeUS: Time since system startup
        // @Field: IMS: GPS INS time (round)
        // @Field: SVs: number of satellites tracked
        // @Field: SolSVs: number of satellites used in solution
        // @Field: SolL1: number of satellites with L1/E1/B1 signals used in solution
        // @Field: SolMult: number of satellites with multi-frequency signals used in solution
        // @Field: SU1: Galileo and BeiDou signal-used mask
        // @Field: SU2: GPS and GLONASS signal-used mask
        // @Field: TimeS: GPS time status
        // @Field: SolS: Extended solution status

        AP::logger().WriteStreaming("ILB6", "TimeUS,IMS,SVs,SolSVs,SolL1,SolMult,SU1,SU2,GTimeS,SolS",
                                    "s---------",
                                    "F---------",
                                    "QIBBBBBBBB",
                                    now_us, sensors_data.ins.ms_tow,
                                    sensors_data.gps.full_sat_info.SVs, sensors_data.gps.full_sat_info.SolnSVs,
                                    sensors_data.gps.full_sat_info.SolnL1SVs, sensors_data.gps.full_sat_info.SolnMultiSVs,
                                    sensors_data.gps.full_sat_info.signal_used1, sensors_data.gps.full_sat_info.signal_used2,
                                    sensors_data.gps.full_sat_info.GPS_time_status, sensors_data.gps.full_sat_info.ext_sol_status);
    }

    if ((sensors_data.ext.new_aiding_data & (NewAidingData::NEW_EXT_POS |
                                          NewAidingData::NEW_EXT_HOR_POS |
                                          NewAidingData::NEW_ALTITUDE |
                                          NewAidingData::NEW_HEADING)) != 0) {
        // @LoggerMessage: ILB7
        // @Description: InertialLabs aiding data1
        // @Field: TimeUS: Time since system startup
        // @Field: IMS: GPS INS time (round)
        // @Field: Lat: Latitude external
        // @Field: Lng: Longitude external
        // @Field: Alt: Altitude external
        // @Field: LatS: Latitude external STD
        // @Field: LngS: Longitude external STD
        // @Field: AltS: Altitude external STD
        // @Field: PosL: External position latency
        // @Field: Yaw: Heading external
        // @Field: YawS: Heading external STD
        // @Field: YawL: Heading external latency

        AP::logger().WriteStreaming("ILB7", "TimeUS,IMS,Lat,Lng,Alt,LatS,LngS,AltS,PosL,Yaw,YawS,YawL",
                                    "s-DUmmmm-hh-",
                                    "F-----------",
                                    "QIddffffffff",
                                    now_us, sensors_data.ins.ms_tow,
                                    static_cast<double>(sensors_data.ext.hor_pos.lat)*1.0e-7,
                                    static_cast<double>(sensors_data.ext.hor_pos.lon)*1.0e-7,
                                    static_cast<float>(sensors_data.ext.alt.alt)*1.0e-3f,
                                    static_cast<float>(sensors_data.ext.hor_pos.lat_std)*0.01f,
                                    static_cast<float>(sensors_data.ext.hor_pos.lon_std)*0.01f,
                                    static_cast<float>(sensors_data.ext.alt.alt_std)*0.01f,
                                    static_cast<float>(sensors_data.ext.hor_pos.pos_latency)*1.0e-3f,
                                    static_cast<float>(sensors_data.ext.heading.heading)*0.01f,
                                    static_cast<float>(sensors_data.ext.heading.std)*0.01f,
                                    static_cast<float>(sensors_data.ext.heading.latency)*1.0e-3f);
    }

    if ((sensors_data.ext.new_aiding_data & (NewAidingData::NEW_AMBIENT |
                                          NewAidingData::NEW_WIND |
                                          NewAidingData::NEW_AIRSPEED)) != 0) {
        // @LoggerMessage: ILB8
        // @Description: InertialLabs aiding data2
        // @Field: TimeUS: Time since system startup
        // @Field: IMS: GPS INS time (round)
        // @Field: Spd: External air or ground speed
        // @Field: Temp: External temperature
        // @Field: Alt: External altitude
        // @Field: Press: External pressure
        // @Field: WN: External North wind component
        // @Field: WE: External East wind component
        // @Field: WNS: External North wind STD
        // @Field: WES: External East wind component

        AP::logger().WriteStreaming("ILB8", "TimeUS,IMS,Spd,Temp,Alt,Press,WN,WE,WNS,WES",
                                    "s-nOmPnnnn",
                                    "F---------",
                                    "QIffffffff",
                                    now_us, sensors_data.ins.ms_tow,
                                    static_cast<float>(sensors_data.ext.speed)*0.5144f*0.01f,
                                    static_cast<float>(sensors_data.ext.ambient_air_data.air_temp)*0.1f,
                                    static_cast<float>(sensors_data.ext.ambient_air_data.alt)*0.01f,
                                    static_cast<float>(sensors_data.ext.ambient_air_data.abs_press)*2.0f,
                                    static_cast<float>(sensors_data.ext.wind_data.e_wind_vel)*0.5144f*0.01f,
                                    static_cast<float>(sensors_data.ext.wind_data.n_wind_vel)*0.5144f*0.01f,
                                    static_cast<float>(sensors_data.ext.wind_data.e_std_wind)*0.5144f*0.01f,
                                    static_cast<float>(sensors_data.ext.wind_data.n_std_wind)*0.5144f*0.01f);
    }
#endif  // HAL_LOGGING_ENABLED
}

void AP_ExternalAHRS_InertialLabs::send_data_to_sensor()
{
    const bool transmit_airspeed = option_is_set(AP_ExternalAHRS::OPTIONS::ILAB_TRANSMIT_AIRSPEED);
    if (transmit_airspeed) {
        const uint16_t inu_data_rate = get_rate(); // Hz
        const uint16_t max_aiding_data_rate = 50; // Hz
        uint16_t ticks_for_one_send = (inu_data_rate / max_aiding_data_rate);
        if (inu_data_rate % max_aiding_data_rate)
        {
            ++ticks_for_one_send;
        }

        if (airspeed_message_counter < ticks_for_one_send)
        {
            ++airspeed_message_counter;
        }
        else
        {
            send_airspeed_aiding_data();
            airspeed_message_counter = 0;
        }
    }
}

void AP_ExternalAHRS_InertialLabs::send_GCS_messages()
{
    using InertialLabs::USW;
    using InertialLabs::adu_message_list;
    using InertialLabs::adu_message_list_size;
    using InertialLabs::usw_message_list;
    using InertialLabs::usw_message_list_size;
    using InertialLabs::usw2_message_list;
    using InertialLabs::usw2_message_list_size;

    const bool need_send = option_is_set(AP_ExternalAHRS::OPTIONS::ILAB_SEND_STATUS);
    if (!need_send)
    {
        return;
    }

    const InertialLabs::SensorsData &sensors_data = sensor.get_sensors_data();

    // Send IL INS status messages to GCS via MAVLink
    if (sensors_data.ins.unit_status != last_sensor_data.unit_status) {
        send_EAHRS_status_msg(last_sensor_data.unit_status,
                              sensors_data.ins.unit_status,
                              usw_message_list,
                              usw_message_list_size,
                              usw_last_message_timestamp_ms); // IL INS Unit Status Word (USW) messages
        last_sensor_data.unit_status = sensors_data.ins.unit_status;

        if ((sensors_data.ins.unit_status & USW::MAG_VG3D_CLB_RUNTIME) != 0) {
            if ((last_sensor_data.mag_clb_status & (1 << 0)) == 0) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: VG3D Mag calibration data accumulation");
                last_sensor_data.mag_clb_status |= (1 << 0); // set bit: VG3D mag calibration data is accumulated
            } else {
                last_sensor_data.mag_clb_status |= (1 << 1); // set bit: VG3D mag calibration parameters are calculated
                last_sensor_data.mag_clb_status &= ~(1 << 0);
                last_sensor_data.mag_ms = sensors_data.package_timestamp_ms;
            }
        }

        if ((sensors_data.ins.unit_status & USW::MAG_VG3D_CLB_SUCCESS) != 0) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: VG3D Mag calibration successful");
            last_sensor_data.mag_clb_status |= (1 << 2); // set bit: VG3D mag calibration accuracy estimation in progress
            last_sensor_data.mag_clb_status &= ~(1 << 1);
        }

        if ((last_sensor_data.mag_clb_status & (1 << 1)) != 0 && (last_sensor_data.unit_status & USW::MAG_VG3D_CLB_RUNTIME) == 0) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ILAB: VG3D Mag calibration unsuccessful");
            last_sensor_data.mag_clb_status &= ~(1 << 1);
        }
    }

    if ((last_sensor_data.mag_clb_status & (1 << 2)) != 0) {
        const bool is_time_exceeded = (sensors_data.package_timestamp_ms > last_sensor_data.mag_ms) && (sensors_data.package_timestamp_ms - last_sensor_data.mag_ms > 10000U);
        const bool is_accuracy_changed = sensors_data.ins.mag_clb_accuracy != last_sensor_data.mag_clb_accuracy;
        if (is_time_exceeded || is_accuracy_changed) {
            if (sensors_data.ins.mag_clb_accuracy == 255) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: INS cannot estimate heading accuracy");
            } else if (sensors_data.ins.mag_clb_accuracy != 0) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: Predicted heading error is %.1f deg", static_cast<float>(sensors_data.ins.mag_clb_accuracy)*0.1f);
            }
            last_sensor_data.mag_clb_status &= ~(1 << 2);
            last_sensor_data.mag_clb_accuracy = sensors_data.ins.mag_clb_accuracy;
        }
    }

    if (sensors_data.ins.unit_status2 != last_sensor_data.unit_status2) {
        send_EAHRS_status_msg(last_sensor_data.unit_status2,
                                sensors_data.ins.unit_status2,
                                usw2_message_list,
                                usw2_message_list_size,
                                usw2_last_message_timestamp_ms); // IL INS Unit Status Word 2 (USW2) messages
        last_sensor_data.unit_status2 = sensors_data.ins.unit_status2;
    }

    if (sensors_data.ins.air_data_status != last_sensor_data.air_data_status) {
        send_EAHRS_status_msg(last_sensor_data.air_data_status,
                                sensors_data.ins.air_data_status,
                                adu_message_list,
                                adu_message_list_size,
                                adu_last_message_timestamp_ms); // IL Air Data Unit (ADU) messages
        last_sensor_data.air_data_status = sensors_data.ins.air_data_status;
    }

    if (last_sensor_data.spoof_status != sensors_data.gps.spoof_status) {
        // IL INS spoofing detection messages
        if ((last_sensor_data.spoof_status == 2 || last_sensor_data.spoof_status == 3) && (sensors_data.gps.spoof_status == 1)) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: GNSS no spoofing");
        }

        if (last_sensor_data.spoof_status == 2) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: GNSS spoofing indicated");
        }

        if (last_sensor_data.spoof_status == 3) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: GNSS multiple spoofing indicated");
        }

        last_sensor_data.spoof_status = sensors_data.gps.spoof_status;
    }

    if (last_sensor_data.jam_status != sensors_data.gps.jam_status) {
        // IL INS jamming detection messages
        if ((last_sensor_data.jam_status == 3) && (sensors_data.gps.jam_status == 1)) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: GNSS no jamming");
        }

        if (sensors_data.gps.jam_status == 3) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: GNSS jamming indicated and no fix");
        }

        last_sensor_data.jam_status = sensors_data.gps.jam_status;
    }

    if (last_sensor_data.ins_sol_status != sensors_data.ins.ins_sol_status) {
        // IL INS navigation solution status messages
        if ((last_sensor_data.ins_sol_status == 4 ||
                last_sensor_data.ins_sol_status == 6 ||
                last_sensor_data.ins_sol_status == 8) &&
                sensors_data.ins.ins_sol_status == 0) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: INS solution is good");
        }

        if ((sensors_data.ins.ins_sol_status) == 4 && (last_sensor_data.ins_sol_status == 0)) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: INS is operating in autonomous mode");
        }

        if (sensors_data.ins.ins_sol_status == 6) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: INS froze position and velocity");
        }

        if (sensors_data.ins.ins_sol_status == 8) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "ILAB: INS solution is invalid");
        }

        last_sensor_data.ins_sol_status = sensors_data.ins.ins_sol_status;
    }
}

// send INS status to GCS via MAVLink
void AP_ExternalAHRS_InertialLabs::send_EAHRS_status_msg(uint16_t last_state,
                                                         uint16_t current_state,
                                                         const InertialLabs::StatusMessage* message_list,
                                                         const size_t message_list_size,
                                                         uint64_t* last_msg_ms)
{
    const uint64_t send_critical_messaged_delay = 10000;

    const InertialLabs::SensorsData &sensors_data = sensor.get_sensors_data();

    for (size_t i = 0; i < message_list_size; i++) {
        const bool current_status = current_state & message_list[i].status;
        const bool last_status = last_state & message_list[i].status;
        const bool is_status_changed = current_status != last_status;

        if (message_list[i].severity == MAV_SEVERITY_CRITICAL &&
                current_status &&
                (is_status_changed || (sensors_data.package_timestamp_ms - last_msg_ms[i] > send_critical_messaged_delay))) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "ILAB: %s", message_list[i].message_true);
            last_msg_ms[i] = sensors_data.package_timestamp_ms;
            continue;
        }

        if (is_status_changed) {
            if (current_status) {
                GCS_SEND_TEXT(message_list[i].severity, "ILAB: %s", message_list[i].message_true);
            } else {
                GCS_SEND_TEXT(message_list[i].severity, "ILAB: %s", message_list[i].message_false);
            }
        }
    }
}

void AP_ExternalAHRS_InertialLabs::send_airspeed_aiding_data()
{
    InertialLabs::Data_context context;
    ExternalAHRS_command_data data;

    data.param1 = floorf(AP::airspeed()->get_raw_airspeed() * 1.94384449f * 100.0f + 0.5f); // in 0.01 kt
    if (data.param1 > 32267.0f)
    {
        data.param1 = 32267.0f;
    }
    else if (data.param1 < -32268.0f)
    {
        data.param1 = -32268.0f;
    }

    InertialLabs::fill_command_pyload(context,
                                      ExternalAHRS_command::AIDING_DATA_AIR_SPEED,
                                      data);
    InertialLabs::fill_transport_protocol_data(context);
    write_bytes(reinterpret_cast<const char *>(context.data), context.length);
}

#endif  // AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED
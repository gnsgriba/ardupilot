#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Math/vector3.h>

namespace InertialLabs {

struct PACKED vec3_16_t {
    int16_t x,y,z;
    Vector3f tofloat(void) {
        return Vector3f(x,y,z);
    }
};

struct PACKED vec3_32_t {
    int32_t x,y,z;
    Vector3f tofloat(void) {
        return Vector3f(x,y,z);
    }
};

enum DataType : uint8_t {
    GPS_INS_TIME_MS = 0x01,
    GPS_WEEK = 0x3C,
    ACCEL_DATA_HR = 0x23,
    GYRO_DATA_HR = 0x21,
    BARO_DATA = 0x25,
    MAG_DATA = 0x24,
    SENSOR_BIAS = 0x26,
    ORIENTATION_ANGLES = 0x07,
    VELOCITIES = 0x12,
    POSITION = 0x10,
    UNIT_STATUS = 0x53,
    GNSS_EXTENDED_INFO = 0x4A,
    GNSS_POSITION = 0x30,
    GNSS_VEL_TRACK = 0x32,
    GNSS_POS_TIMESTAMP = 0x3E,
    GNSS_NEW_DATA = 0x41,
    GNSS_JAM_STATUS = 0xC0,
    DIFFERENTIAL_PRESSURE = 0x28,
    TRUE_AIRSPEED = 0x86,
    CALIBRATED_AIRSPEED = 0x85,
    WIND_SPEED = 0x8A,
    AIR_DATA_STATUS = 0x8D,
    SUPPLY_VOLTAGE = 0x50,
    TEMPERATURE = 0x52,
    UNIT_STATUS2 = 0x5A,
    GNSS_DOP = 0x42,
    INS_SOLUTION_STATUS = 0x54,
    INS_POS_VEL_ACCURACY = 0x5F,
    FULL_SAT_INFO = 0x37,
    GNSS_VEL_LATENCY = 0x3D,
    GNSS_SOL_STATUS = 0x38,
    GNSS_POS_VEL_TYPE = 0x39,
    NEW_AIDING_DATA = 0x65,
    NEW_AIDING_DATA2 = 0xA1,
    EXT_SPEED = 0x61,
    EXT_HOR_POS = 0x6E,
    EXT_ALT = 0x6C,
    EXT_HEADING = 0x66,
    EXT_AMBIENT_DATA = 0x6B,
    EXT_WIND_DATA = 0x62,
    MAG_CLB_ACCURACY = 0x9A,
    DOPPLER_VELOCITY_LOG = 0X67,
};

// Ins Solutiob Status
enum InsSolution {
    GOOD                       = 0,
    KF_CONVERGENCE_IN_PROGRESS = 1,
    GNSS_ABSENCE               = 2,
    NO_HEADING_CORRECTION      = 3,
    AUTONOMOUS_MODE            = 4,
    NO_GNSS_AIDING_DATA        = 5,
    AUTONIMUS_TIME_EXCEEDED    = 6,
    ZUPT_MODE                  = 7,
    INVALID                    = 8,
};

// Unit Status Word bits
enum USW {
    INITIAL_ALIGNMENT_FAIL	= (1U << 0),
    OPERATION_FAIL			= (1U << 1),
    GYRO_FAIL				= (1U << 2),
    ACCEL_FAIL				= (1U << 3),
    MAG_FAIL				= (1U << 4),
    ELECTRONICS_FAIL		= (1U << 5),
    GNSS_FAIL				= (1U << 6),
    MAG_VG3D_CLB_RUNTIME	= (1U << 7),
    VOLTAGE_LOW				= (1U << 8),
    VOLTAGE_HIGH			= (1U << 9),
    GYRO_X_RATE_HIGH		= (1U << 10),
    GYRO_Y_RATE_HIGH	    = (1U << 11),
    GYRO_Z_RATE_HIGH		= (1U << 12),
    MAG_FIELD_HIGH			= (1U << 13),
    TEMP_RANGE_ERR			= (1U << 14),
    MAG_VG3D_CLB_SUCCESS	= (1U << 15)
};

// Unit Status Word2 bits
enum USW2 {
    ACCEL_X_HIGH           	 = (1U << 0),
    ACCEL_Y_HIGH           	 = (1U << 1),
    ACCEL_Z_HIGH           	 = (1U << 2),
    ADU_BARO_FAIL            = (1U << 3),
    ADU_DIFF_PRESS_FAIL   	 = (1U << 4),
    MAG_AUTO_CAL_2D_RUNTIME  = (1U << 5),
    MAG_AUTO_CAL_3D_RUNTIME  = (1U << 6),
    GNSS_FUSION_OFF        	 = (1U << 7),
    DIFF_PRESS_FUSION_OFF  	 = (1U << 8),
    GNSS_POS_VALID           = (1U << 10)
};

// Air Data Status bits
enum ADU {
    BARO_INIT_FAIL            = (1U << 0),
    DIFF_PRESS_INIT_FAIL      = (1U << 1),
    BARO_FAIL       		  = (1U << 2),
    DIFF_PRESS_FAIL           = (1U << 3),
    BARO_RANGE_ERR  		  = (1U << 4),
    DIFF_PRESS_RANGE_ERR      = (1U << 5),
    BARO_ALT_FAIL             = (1U << 8),
    AIRSPEED_FAIL             = (1U << 9),
    AIRSPEED_BELOW_THRESHOLD  = (1U << 10)
};

// New GPS data indicator
enum NewGPSData {
    NEW_GNSS_POSITION     = (1U << 0),
    NEW_GNSS_VELOCITY     = (1U << 1),
    NEW_GNSS_HEADING      = (1U << 2),
    NEW_VALID_PPS         = (1U << 3),
    NEW_GNSS_BESTXYZ_LOG  = (1U << 4),
    NEW_GNSS_PSRDOP_LOG   = (1U << 5),
    NEW_PPS               = (1U << 6),
    NEW_RANGE_LOG         = (1U << 7)
};

// New aiding data indicator
enum NewAidingData {
    NEW_AIRSPEED    = (1U << 1),
    NEW_WIND        = (1U << 2),
    NEW_EXT_POS     = (1U << 3),
    NEW_HEADING     = (1U << 5),
    NEW_DVL         = (1U << 6),
    NEW_AMBIENT     = (1U << 10),
    NEW_ALTITUDE    = (1U << 11),
    NEW_EXT_HOR_POS = (1U << 13),
    NEW_ADC         = (1U << 15)
};

enum JammingStatus : uint8_t {
    UNKOWN_OR_DISABLED = 0,
    OK                 = 1,  // No significant jamming
    WARNING            = 2,  // Interference visible but fix ok
    CRITICAL           = 3,  // Interference visible and no fix
};

enum SpoofingStatus : uint8_t {
    UNKOWN_OR_DEACTIVATED = 0,
    NO                    = 1,
    INDICATED             = 2,
    MULTIPLE_INCIDATIONS  = 3,
};

/*
    Every data package consist of:
        MessageHeader (6 bytes)
        Payload
        Checksum (2 bytes)

    Whole data package size = MessageHeader->msgLen + 2(0x55AA)

    User Defined Data payload consist of:
        Number of data structures
        List of data structures ID
        List of data structures Values

*/
constexpr uint16_t PACKAGE_HEADER = 0x55AA;

struct PACKED MessageHeader {
    uint16_t package_header;  // 0x55AA
    uint8_t msg_type;     // always 1 for INS data
    uint8_t msg_id;       // always 0x95
    uint16_t msg_len;     // payload + 6(msgType + msgId + msgLen + checksum). Not included packageHeader
};

struct PACKED SensorBias {
    int8_t gyroX;  // deg/s*0.5*1e5
    int8_t gyroY;  // deg/s*0.5*1e5
    int8_t gyroZ;  // deg/s*0.5*1e5
    int8_t accX;   // g*0.5*1e6
    int8_t accY;   // g*0.5*1e6
    int8_t accZ;   // g*0.5*1e6
    int8_t reserved;
};

struct PACKED GnssExtendedInfo {
    uint8_t fix_type;
    uint8_t spoofing_status;
};

struct PACKED FullSatInfo {
    uint8_t SVs;
    uint8_t SolnSVs;
    uint8_t SolnL1SVs;
    uint8_t SolnMultiSVs;
    uint8_t signal_used1;
    uint8_t signal_used2;
    uint8_t GPS_time_status;
    uint8_t ext_sol_status;
};

struct PACKED GnssDop {
    uint16_t gdop;  // *1000
    uint16_t pdop;  // *1000
    uint16_t hdop;  // *1000
    uint16_t vdop;  // *1000
    uint16_t tdop;  // *1000
};

struct PACKED InsAccuracy {
    int32_t lat;       // m*1000
    int32_t lon;       // m*1000
    int32_t alt;       // m*1000
    int32_t east_vel;  // m/s*1000
    int32_t north_vel; // m/s*1000
    int32_t ver_vel;   // m/s*1000
};

struct PACKED ExtHorPos {
    int32_t lon;          // deg*1.0e7
    int32_t lat;          // deg*1.0e7
    uint16_t lat_std;     // m*100
    uint16_t lon_std;     // m*100
    uint16_t pos_latency; // sec*1000
};

struct PACKED ExtAlt {
    int32_t alt;       // m*1000, AMSL
    uint16_t alt_std;  // m*100
};

struct PACKED ExtHeading {
    uint16_t heading;  // deg*100
    uint16_t std;      // deg*100
    uint16_t latency;  // sec*1000
};

struct PACKED ExtAmbientData {
    int16_t air_temp;    // degC*10
    int32_t alt;         // m*100, AMSL
    uint16_t abs_press;  // Pa/2
};

struct PACKED ExtWindData {
    int16_t n_wind_vel;   // kt*100
    int16_t e_wind_vel;   // kt*100
    uint16_t n_std_wind;  // kt*100
    uint16_t e_std_wind;  // kt*100
};

struct PACKED DopplerVelocityLog {
    int32_t lateralVel; // m/sec*1000
    int32_t forwardVel; // m/sec*1000
    int32_t verticalVel; // m/sec*1000
    uint16_t lateralVelStd; // m/sec*1000
    uint16_t forwardVelStd; // m/sec*1000
    uint16_t verticalVelStd; // m/sec*1000
    uint16_t velLatency; // ms
    int32_t reserved;
};

union PACKED UDDMessageData {
    uint32_t gps_time_ms;  // ms since start of GPS week
    uint16_t gps_week;
    vec3_32_t accel_data_hr;  // g * 1e6
    vec3_32_t gyro_data_hr;   // deg/s * 1e5
    struct PACKED {
        uint16_t pressure_pa2;  // Pascals/2
        int32_t baro_alt;       // meters*100, AMSL
    } baro_data;
    vec3_16_t mag_data;  // nT/10
    SensorBias sensor_bias;
    struct PACKED {
        uint16_t yaw;      // deg*100
        int16_t pitch;     // deg*100
        int16_t roll;      // deg*100
    } orientation_angles;  // 321 euler order?
    vec3_32_t velocity;    // m/s * 100
    struct PACKED {
        int32_t lat;  // deg*1e7
        int32_t lon;  // deg*1e7
        int32_t alt;  // m*100, AMSL
    } position;
    uint16_t unit_status; // set ILABS_UNIT_STATUS_*
    GnssExtendedInfo gnss_extended_info;
    struct PACKED {
        int32_t lat;  // deg*1e7
        int32_t lon;  // deg*1e7
        int32_t alt;  // m*100
    } gnss_position;
    struct PACKED {
        int32_t hor_speed;           // m/s*100
        uint16_t track_over_ground;  // deg*100
        int32_t ver_speed;           // m/s*100
    } gnss_vel_track;
    uint32_t gnss_pos_timestamp;  // ms
    uint8_t gnss_new_data;
    uint8_t gnss_jam_status;
    DopplerVelocityLog doppler_velocity_log;
    int32_t differential_pressure;  // mbar*1e4
    int16_t true_airspeed;          // m/s*100
    int16_t calibrated_airspeed;    // m/s*100
    vec3_16_t wind_speed;           // m/s*100
    uint16_t air_data_status;
    uint16_t supply_voltage;        // V*100
    int16_t temperature;            // degC*10
    uint16_t unit_status2;
    GnssDop gnss_dop;
    uint8_t ins_sol_status;
    InsAccuracy ins_accuracy;
    FullSatInfo full_sat_info;
    uint16_t gnss_vel_latency;      // ms
    uint8_t gnss_sol_status;
    uint8_t gnss_pos_vel_type;
    uint16_t new_aiding_data;
    uint16_t new_aiding_data2;
    int16_t external_speed;         // kt*100
    ExtHorPos ext_hor_pos;
    ExtAlt ext_alt;
    ExtHeading ext_heading;
    ExtAmbientData ext_ambient_air_data;
    ExtWindData ext_wind_data;
    uint8_t mag_clb_accuracy;       // deg*10
};

struct GpsData {
    uint32_t ms_tow;
    uint16_t gps_week;
    int32_t latitude;         // deg*1.0e7
    int32_t longitude;        // deg*1.0e7
    int32_t altitude;         // cm
    float hor_speed;          // m/s
    float ver_speed;          // m/s
    float track_over_ground;  // deg
    GnssDop dop;
    uint8_t new_data;
    uint8_t fix_type;
    uint8_t spoof_status;
    uint8_t jam_status;
    FullSatInfo full_sat_info;
    uint16_t vel_latency;
    uint8_t gnss_sol_status;
    uint8_t gnss_pos_vel_type;
};

struct InsData{
    float yaw;    // deg
    float pitch;  // deg
    float roll;   // deg
    uint32_t ms_tow;
    int32_t latitude;   // deg*1.0e7
    int32_t longitude;  // deg*1.0e7
    int32_t altitude;   // cm
    Vector3f velocity;  // NED, m/s
    uint16_t unit_status;
    uint16_t unit_status2;
    uint8_t ins_sol_status;
    InsAccuracy ins_accuracy;
    float baro_alt;             // m
    float true_airspeed;        // m/s
    float calibrated_airspeed;  // m/s
    Vector3f wind_speed;        // NED, m/s
    float airspeed_sf;
    uint16_t air_data_status;
    SensorBias sensor_bias;
    uint8_t mag_clb_accuracy;
};

struct ExtData {
    uint16_t new_aiding_data;
    uint16_t new_aiding_data2;
    int16_t speed;
    ExtHorPos hor_pos;
    ExtAlt alt;
    ExtHeading heading;
    ExtAmbientData ambient_air_data;
    ExtWindData wind_data;
    DopplerVelocityLog doppler_velocity_log;
};

struct SensorsData {
    uint32_t package_timestamp_ms;
    Vector3f accel;        // NED, m/s^2
    Vector3f gyro;         // NED, rad/s
    Vector3f mag;          // NED, milligauss
    InsData ins{};
    GpsData gps{};
    ExtData ext{};
    float pressure;        // Pa
    float diff_press;      // Pa
    float temperature;     // degC
    float supply_voltage;  // V
};

} // namespace Inertiallabs

#endif  // AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED

#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED

#include <GCS_MAVLink/GCS.h>

namespace InertialLabs {

struct StatusMessage {
	unsigned int status;
	MAV_SEVERITY severity;
	const char* message_true;
	const char* message_false;
};

const StatusMessage usw_message_list[] = {
	{ INITIAL_ALIGNMENT_FAIL, MAV_SEVERITY_CRITICAL, "Unsuccessful initial alignment",        "Initial alignment is OK"          },
	{ OPERATION_FAIL,         MAV_SEVERITY_CRITICAL, "IMU data are incorrect",                "IMU data are correct"             },
	{ GYRO_FAIL,              MAV_SEVERITY_CRITICAL, "Gyros failure",                         "Gyros is OK"                      },
	{ ACCEL_FAIL,             MAV_SEVERITY_CRITICAL, "Accelerometers failure",                "Accelerometers is OK"             },
	{ MAG_FAIL,               MAV_SEVERITY_CRITICAL, "Magnetometers failure",                 "Magnetometers is OK"              },
	{ ELECTRONICS_FAIL,       MAV_SEVERITY_CRITICAL, "Electronics failure",                   "Electronics is OK"                },
	{ GNSS_FAIL,              MAV_SEVERITY_CRITICAL, "GNSS receiver failure",                 "GNSS receiver is OK"              },
	{ VOLTAGE_LOW,            MAV_SEVERITY_WARNING,  "Low input voltage",                     "Input voltage is in range"        },
	{ VOLTAGE_HIGH,           MAV_SEVERITY_WARNING,  "High input voltage",                    "Input voltage is in range"        },
	{ GYRO_X_RATE_HIGH,       MAV_SEVERITY_INFO,     "Y-axis angular rate is exceeded",       "Y-axis angular rate is in range"  },
	{ GYRO_Y_RATE_HIGH,       MAV_SEVERITY_INFO,     "X-axis angular rate is exceeded",       "X-axis angular rate is in range"  },
	{ GYRO_Z_RATE_HIGH,       MAV_SEVERITY_INFO,     "Z-axis angular rate is exceeded",       "Z-axis angular rate is in range"  },
	{ MAG_FIELD_HIGH,         MAV_SEVERITY_INFO,     "Large total magnetic field",            "Total magnetic field is in range" },
	{ TEMP_RANGE_ERR,         MAV_SEVERITY_INFO,     "Temperature is out of range",           "Temperature is in range"          }
};

const StatusMessage usw2_message_list[] = {
	{ ACCEL_X_HIGH,            MAV_SEVERITY_INFO,     "Y-axis acceleration is out of range",     "Y-axis acceleration is in range"     },
	{ ACCEL_Y_HIGH,            MAV_SEVERITY_INFO,     "X-axis acceleration is out of range",     "X-axis acceleration is in range"     },
	{ ACCEL_Z_HIGH,            MAV_SEVERITY_INFO,     "Z-axis acceleration is out of range",     "Z-axis acceleration is in range"     },
	{ ADU_BARO_FAIL,           MAV_SEVERITY_CRITICAL, "Baro altimeter failure",                  "Baro altimeter is OK"                },
	{ ADU_DIFF_PRESS_FAIL,     MAV_SEVERITY_CRITICAL, "Diff. pressure sensor failure",           "Diff. pressure sensor is OK"         },
	{ MAG_AUTO_CAL_2D_RUNTIME, MAV_SEVERITY_INFO,     "Automatic 2D calibration is in progress", "Automatic 2D calibration is stopped" },
	{ MAG_AUTO_CAL_3D_RUNTIME, MAV_SEVERITY_INFO,     "Automatic 3D calibration is in progress", "Automatic 3D calibration is stopped" },
	{ GNSS_FUSION_OFF,         MAV_SEVERITY_INFO,     "GNSS input switched off",                 "GNSS input switched on"              },
	{ DIFF_PRESS_FUSION_OFF,   MAV_SEVERITY_INFO,     "Diff. pressure input switched off",       "Diff. pressure input switched on"    },
	{ GNSS_POS_VALID,          MAV_SEVERITY_INFO,     "Incorrect GNSS position",                 "GNSS position is correct"            }
};

const StatusMessage adu_message_list[] = {
	{ BARO_INIT_FAIL,           MAV_SEVERITY_WARNING, "Static pressure sensor unsuccessful initialization", "Static pressure sensor initialization successful"},
	{ DIFF_PRESS_INIT_FAIL,     MAV_SEVERITY_WARNING, "Diff. pressure sensor unsuccessful initialization",  "Diff. pressure sensor initialization successful"},
	{ BARO_RANGE_ERR,           MAV_SEVERITY_INFO,    "Static pressure is out of range",                    "Static pressure is in range"      },
	{ DIFF_PRESS_RANGE_ERR,     MAV_SEVERITY_INFO,    "Diff. pressure is out of range",                     "Diff. pressure is in range"       },
	{ BARO_ALT_FAIL,            MAV_SEVERITY_WARNING, "Pressure altitude is incorrect",                     "Pressure altitude is correct"     },
	{ AIRSPEED_FAIL,            MAV_SEVERITY_WARNING, "Air speed is incorrect",                             "Air speed is correct"             },
	{ AIRSPEED_BELOW_THRESHOLD, MAV_SEVERITY_INFO,    "Air speed is below the threshold",                   "Air speed is above the threshold" }
};

const size_t usw_message_list_size = sizeof(usw_message_list) / sizeof(StatusMessage);
const size_t usw2_message_list_size = sizeof(usw2_message_list) / sizeof(StatusMessage);
const size_t adu_message_list_size = sizeof(adu_message_list) / sizeof(StatusMessage);

} // namespace InertialLabs

#endif  // AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED
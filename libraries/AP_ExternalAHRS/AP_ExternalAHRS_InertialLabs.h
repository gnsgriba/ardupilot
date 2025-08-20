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
  support for serial connected InertialLabs INS system
 */

#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED

#include "AP_ExternalAHRS_backend.h"
#include "InertialLabs_data.h"
#include "InertialLabs_message_list.h"
#include "InertialLabs_sensor.h"

class AP_ExternalAHRS_InertialLabs : public AP_ExternalAHRS_backend {

public:
    AP_ExternalAHRS_InertialLabs(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state);
    int8_t get_port() const override;  // return -1 if disabled
    bool healthy() const override;
    bool initialised() const override;
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;
    void get_filter_status(nav_filter_status &status) const override;
    bool get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const override;
    void write_bytes(const char *bytes, uint8_t len) override;
    void handle_command(ExternalAHRS_command command, const ExternalAHRS_command_data &data) override;
    bool get_wind_estimation(Vector3f &wind) override;  // < TODO: refactor?
    void send_eahrs_status_flag(class GCS_MAVLINK &link) const override;  // < TODO: refactor
    void update() override;
    const char* get_name() const override { return "ILabs"; }

private:
    uint8_t num_gps_sensors() const override {
        return 1;
    }
    bool handle_full_circle();
    void update_thread();

    void handle_sensor_data();
    void write_logs();
    void send_GCS_messages();
    void send_EAHRS_status_msg(uint16_t last_state,
                               uint16_t current_state,
                               const InertialLabs::StatusMessage* message_list,
                               const size_t message_list_size,
                               uint64_t* last_msg_ms);
    uint16_t get_num_points_to_dec(const uint16_t &rate) const; // < TODO: refactor
    void send_airspeed_packet_to_sensor();  // < TODO: refactor

private:
    InertialLabs::Sensor sensor;

    AP_ExternalAHRS::gps_data_message_t gps_data{};
    AP_ExternalAHRS::mag_data_message_t mag_data{};
    AP_ExternalAHRS::baro_data_message_t baro_data{};
    AP_ExternalAHRS::ins_data_message_t ins_data{};
    AP_ExternalAHRS::airspeed_data_message_t airspeed_data{};

    struct {
        uint16_t unit_status;
        uint16_t unit_status2;
        uint16_t air_data_status;
        uint8_t spoof_status;
        uint8_t jam_status;
        uint8_t ins_sol_status;
        uint8_t mag_clb_status;
        uint8_t mag_clb_accuracy;
        uint32_t att_ms;
        uint32_t vel_ms;
        uint32_t pos_ms;
        uint32_t gps_ms;
        uint32_t mag_ms;
    } last_sensor_data{};

    uint16_t airspeed_message_counter = 0;  // < TODO: refactor
};

#endif  // AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED

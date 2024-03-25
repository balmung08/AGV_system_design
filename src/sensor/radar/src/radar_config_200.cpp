/*
*Copyright (c) 2021 LIU Pony.All rights reserved.
*This software is provided 'as-is', without any express or implied
*warranty. In no event will the author be held liable for any
*damages arising from the use of this software.
*
*Permission is granted to anyone to use this software for any
*purpose, including commercial applications, and to alter it and
*redistribute it freely, subject to the following restrictions:
*
*1. The origin of this software must not be misrepresented; you must
*not claim that you wrote the original software. If you use this
*software in a product, an acknowledgment in the product documentation
*would be appreciated but is not required.
*
*2. Any change that you make about this software should be wrote down. Never change 
*the code easily unless you have a good reason. And the reason must be wrote down.
*
*3. This notice may not be removed or altered from any source
*distribution.
*/
#include "config/radar_config_200.h"

#include "Byte/byte.h"


const uint32_t RadarConfig200::ID = 0x200;

RadarConfig200::RadarConfig200() {
    frame_data=new uint8_t[8];

    max_distance_valid=true;
    sensor_id_valid=false;
    radar_power_valid=false;
    output_type_valid=true;
    send_quality_valid=true;
    send_ext_info_valid=true;
    sort_index_valid=false;
    store_in_nvm_valid=true;
    ctrl_relay_valid=false;
    rcs_threshold_valid=true;

    max_distance=50;
    sensor_id=0;
    output_type=OUTPUT_TYPE_OBJECTS;
    radar_power=0;
    ctrl_relay=0;
    send_ext_info=1;
    send_quality=1;
    sort_index=0;
    store_in_nvm=1;
    rcs_threshold=RCS_THRESHOLD_STANDARD;


}
RadarConfig200::~RadarConfig200() {
    delete frame_data;
    frame_data= nullptr;
}


void RadarConfig200::UpdateData(uint8_t* data) {
    set_max_distance_valid_p(data, RadarConfig200::max_distance_valid);
    set_sensor_id_valid_p(data,RadarConfig200::sensor_id_valid);
    set_radar_power_valid_p(data, RadarConfig200::radar_power_valid);
    set_output_type_valid_p(data, RadarConfig200::output_type_valid);
    set_send_quality_valid_p(data, RadarConfig200::send_quality_valid);
    set_send_ext_info_valid_p(data, RadarConfig200::send_ext_info_valid);
    set_sort_index_valid_p(data, RadarConfig200::sort_index_valid);
    set_store_in_nvm_valid_p(data, RadarConfig200::store_in_nvm_valid);
    set_ctrl_relay_valid_p(data, RadarConfig200::ctrl_relay_valid);
    set_rcs_threshold_valid_p(data, RadarConfig200::rcs_threshold_valid);

    set_max_distance_p(data, RadarConfig200::max_distance);
    set_sensor_id_p(data, RadarConfig200::sensor_id);
    set_output_type_p(data, RadarConfig200::output_type);
    set_radar_power_p(data, RadarConfig200::radar_power);
    set_ctrl_relay_p(data, RadarConfig200::ctrl_relay);
    set_send_ext_info_p(data, RadarConfig200::send_ext_info);
    set_send_quality_p(data, RadarConfig200::send_quality);
    set_sort_index_p(data, RadarConfig200::sort_index);
    set_store_in_nvm_p(data, RadarConfig200::store_in_nvm);
    set_rcs_threshold_p(data, RadarConfig200::rcs_threshold);
}


void RadarConfig200::Reset(uint8_t* data) {

}

void RadarConfig200::set_max_distance_valid_p(uint8_t* data, bool valid) {
    Byte frame(data);
    if (valid) {
        frame.set_bit_1(0);
    } else {
        frame.set_bit_0(0);
    }
}

void RadarConfig200::set_sensor_id_valid_p(uint8_t* data, bool valid) {
    Byte frame(data);
    if (valid) {
        frame.set_bit_1(1);
    } else {
        frame.set_bit_0(1);
    }
}

void RadarConfig200::set_radar_power_valid_p(uint8_t* data, bool valid) {
    Byte frame(data);
    if (valid) {
        frame.set_value(1, 2, 1);
    } else {
        frame.set_value(0, 2, 1);
    }
}

void RadarConfig200::set_output_type_valid_p(uint8_t* data, bool valid) {
    Byte frame(data);
    if (valid) {
        frame.set_value(1, 3, 1);
    } else {
        frame.set_value(0, 3, 1);
    }
}

void RadarConfig200::set_send_quality_valid_p(uint8_t* data, bool valid) {
    Byte frame(data);
    if (valid) {
        frame.set_value(1, 4, 1);
    } else {
        frame.set_value(0, 4, 1);
    }
}

void RadarConfig200::set_send_ext_info_valid_p(uint8_t* data, bool valid) {
    Byte frame(data);
    if (valid) {
        frame.set_value(1, 5, 1);
    } else {
        frame.set_value(0, 5, 1);
    }
}

void RadarConfig200::set_sort_index_valid_p(uint8_t* data, bool valid) {
    Byte frame(data);
    if (valid) {
        frame.set_value(1, 6, 1);
    } else {
        frame.set_value(0, 6, 1);
    }
}

void RadarConfig200::set_store_in_nvm_valid_p(uint8_t* data, bool valid) {
    Byte frame(data);
    if (valid) {
        frame.set_value(1, 7, 1);
    } else {
        frame.set_value(0, 7, 1);
    }
}

void RadarConfig200::set_ctrl_relay_valid_p(uint8_t* data, bool valid) {
    Byte frame(data + 5);
    if (valid) {
        frame.set_bit_1(0);
    } else {
        frame.set_bit_0(0);
    }
}

void RadarConfig200::set_rcs_threshold_valid_p(uint8_t* data, bool valid) {
    Byte frame(data + 6);
    if (valid) {
        frame.set_bit_1(0);
    } else {
        frame.set_bit_0(0);
    }
}

void RadarConfig200::set_max_distance_p(uint8_t* data, uint16_t value) {
    value /= 2;
    uint8_t low = static_cast<uint8_t>(value >> 2);
    Byte frame_low(data + 1);
    frame_low.set_value(low, 0, 8);

    uint8_t high = static_cast<uint8_t>(value << 6);
    high &= 0xc0;
    Byte frame_high(data + 2);
    frame_high.set_value(high, 0, 8);
}

void RadarConfig200::set_sensor_id_p(uint8_t* data, uint8_t value) {
    Byte frame(data + 4);
    frame.set_value(value, 0, 3);
}

void RadarConfig200::set_output_type_p(uint8_t* data, OutputType type) {
    Byte frame(data + 4);
    uint8_t value = static_cast<uint8_t>(type);
    frame.set_value(value, 3, 2);
}

void RadarConfig200::set_radar_power_p(uint8_t* data, uint8_t value) {
    Byte frame(data + 4);
    frame.set_value(value, 5, 3);
}

void RadarConfig200::set_ctrl_relay_p(uint8_t* data, uint8_t value) {
    Byte frame(data + 5);
    frame.set_value(value, 1, 1);
}

void RadarConfig200::set_send_ext_info_p(uint8_t* data, uint8_t value) {
    Byte frame(data + 5);
    frame.set_value(value, 3, 1);
}

void RadarConfig200::set_send_quality_p(uint8_t* data, uint8_t value) {
    Byte frame(data + 5);
    frame.set_value(value, 2, 1);
}

void RadarConfig200::set_sort_index_p(uint8_t* data, uint8_t value) {
    Byte frame(data + 5);
    frame.set_value(value, 4, 3);
}

void RadarConfig200::set_store_in_nvm_p(uint8_t* data, uint8_t value) {
    Byte frame(data + 5);
    frame.set_value(value, 7, 1);
}

void RadarConfig200::set_rcs_threshold_p(uint8_t* data,
                                         RcsThreshold rcs_threshold) {
    Byte frame(data + 6);
    uint8_t value = static_cast<uint8_t>(rcs_threshold);
    frame.set_value(value, 1, 3);
}

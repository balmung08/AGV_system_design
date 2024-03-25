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

#pragma once
#include <stdint.h>


enum OutputType {
    OUTPUT_TYPE_NONE = 0,
    OUTPUT_TYPE_OBJECTS = 1,
    OUTPUT_TYPE_CLUSTERS = 2,
    OUTPUT_TYPE_ERROR = 3
};

enum RcsThreshold {
    RCS_THRESHOLD_STANDARD = 0,
    RCS_THRESHOLD_HIGH_SENSITIVITY = 1,
    RCS_THRESHOLD_ERROR = 2
};



class RadarConfig200
{
public:
	static const uint32_t ID;
	RadarConfig200();
	~RadarConfig200();


	void UpdateData(uint8_t* data) ;

	void Reset(uint8_t* data);

	void set_max_distance_valid_p(uint8_t* data, bool valid);
	void set_sensor_id_valid_p(uint8_t* data, bool valid);
	void set_radar_power_valid_p(uint8_t* data, bool valid);
	void set_output_type_valid_p(uint8_t* data, bool valid);
	void set_send_quality_valid_p(uint8_t* data, bool valid);
	void set_send_ext_info_valid_p(uint8_t* data, bool valid);
	void set_sort_index_valid_p(uint8_t* data, bool valid);
	void set_store_in_nvm_valid_p(uint8_t* data, bool valid);
	void set_ctrl_relay_valid_p(uint8_t* data, bool valid);
	void set_rcs_threshold_valid_p(uint8_t* data, bool valid);

	void set_max_distance_p(uint8_t* data, uint16_t value);
	void set_sensor_id_p(uint8_t* data, uint8_t value);
	void set_output_type_p(uint8_t* data, OutputType type);
	void set_radar_power_p(uint8_t* data, uint8_t value);
	void set_ctrl_relay_p(uint8_t* data, uint8_t value);
	void set_send_ext_info_p(uint8_t* data, uint8_t value);
	void set_send_quality_p(uint8_t* data, uint8_t value);
	void set_sort_index_p(uint8_t* data, uint8_t value);
	void set_store_in_nvm_p(uint8_t* data, uint8_t value);
	void set_rcs_threshold_p(uint8_t* data, RcsThreshold rcs_theshold);

private:
	 uint8_t *frame_data;
	 bool  max_distance_valid;
	 bool  sensor_id_valid;
	 bool  radar_power_valid;
	 bool  output_type_valid;
	 bool  send_quality_valid;
	 bool  send_ext_info_valid;
	 bool  sort_index_valid;
	 bool  store_in_nvm_valid;
	 bool  ctrl_relay_valid;
	 bool  rcs_threshold_valid;

	 uint16_t max_distance;
	 uint8_t sensor_id;
	 OutputType output_type;
	 uint8_t radar_power;
	 uint8_t ctrl_relay;
	 uint8_t send_ext_info;
	 uint8_t send_quality;
	 uint8_t sort_index;
	 uint8_t store_in_nvm;
	 RcsThreshold rcs_threshold;

};

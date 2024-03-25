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

class RadarConfig202
{
public:
        static const uint32_t ID;
	RadarConfig202();
	~RadarConfig202();
	
	
	void UpdateData(uint8_t* data) ;//更新数据
	void Reset(uint8_t* data);
	
	void set_FilterCfg_Type(uint8_t* data, bool value);
	void set_FilterCfg_Index(uint8_t* data, uint8_t value);
	void set_FilterCfg_Valid(uint8_t* data, bool value);
	void set_FilterCfg_active(uint8_t* data, bool value);
	void set_FilterCfg_Min_Distance(uint8_t* data, uint16_t value);
	void set_FilterCfg_Max_Distance(uint8_t* data, uint16_t value);

	
private:
        uint8_t *frame_data;
        bool FilterCfg_Type;
        uint8_t FilterCfg_Index;
        bool FilterCfg_Active;
        bool FilterCfg_Valid;
        uint16_t FilterCfg_Min_Distance;
        uint16_t FilterCfg_Max_Distance;
        
	
};

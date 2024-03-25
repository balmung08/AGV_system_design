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
#include "config/radar_config_202.h"

#include "Byte/byte.h"

const uint32_t RadarConfig202::ID = 0x202;

RadarConfig202::RadarConfig202()
{
	frame_data = new uint8_t[8];
	FilterCfg_Type = 1;
	FilterCfg_Index = 1;
        FilterCfg_Active = 1;
        FilterCfg_Valid = 1;
        FilterCfg_Min_Distance = 0;
        FilterCfg_Max_Distance = 50;
        
}

RadarConfig202::~RadarConfig202() 
{
    delete frame_data;
    frame_data= nullptr;
}


void RadarConfig202::UpdateData(uint8_t* data)
{
    set_FilterCfg_Type(data,FilterCfg_Type);
    set_FilterCfg_Index(data,FilterCfg_Index);
    set_FilterCfg_Valid(data,FilterCfg_Valid);
    set_FilterCfg_Min_Distance(data,FilterCfg_Min_Distance);
    set_FilterCfg_Max_Distance(data,FilterCfg_Max_Distance);
}


void RadarConfig202::Reset(uint8_t* data) 
{

}

void RadarConfig202::set_FilterCfg_Type(uint8_t* data, bool value)
{
    Byte frame(data);
    if (value) {
        frame.set_bit_1(7);
    } else {
        frame.set_bit_0(7);
    }
}

void RadarConfig202::set_FilterCfg_Index(uint8_t* data, uint8_t value)
{
    Byte frame(data);
    frame.set_value(value, 3, 4);
}

void RadarConfig202::set_FilterCfg_Valid(uint8_t* data, bool value)
{
    Byte frame(data);
    if (value) {
        frame.set_bit_1(2);
    } else {
        frame.set_bit_0(2);
    }
}
	
void RadarConfig202::set_FilterCfg_active(uint8_t* data, bool value)
{
    Byte frame(data);
    if (value) {
        frame.set_bit_1(1);
    } else {
        frame.set_bit_0(1);
    }
}

void RadarConfig202::set_FilterCfg_Min_Distance(uint8_t* data, uint16_t value)
{
    value*=10;
    uint8_t low = static_cast<uint8_t>(value/256);
    low = low & 0x0F;
    Byte frame_low(data + 1);
    frame_low.set_value(low, 0, 8);
    
    uint8_t high = static_cast<uint8_t>(value%256);
    Byte frame_high(data + 2);
    frame_high.set_value(low, 0, 8);
}

void RadarConfig202::set_FilterCfg_Max_Distance(uint8_t* data, uint16_t value)
{
    value*=10;
    uint8_t low = static_cast<uint8_t>(value/256);
    low = low & 0x0F;
    Byte frame_low(data + 3);
    frame_low.set_value(low, 0, 8);
    
    uint8_t high = static_cast<uint8_t>(value%256);
    Byte frame_high(data + 4);
    frame_high.set_value(low, 0, 8);
}











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

#include "config/motion_input_speed_300.h"

#include "Byte/byte.h"



MotionInputSpeed300::MotionInputSpeed300() {}
MotionInputSpeed300::~MotionInputSpeed300() {}

uint32_t MotionInputSpeed300::GetPeriod(){
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

/**
 * @brief update the data
 * @param data a pointer to the data to be updated
 */
void MotionInputSpeed300::UpdateData(uint8_t* data) 
{
  if (std::isnan(speed_)) 
  {
    //AWARN << "speed is nan";
    return;
  }
  uint32_t speed_direction = 0;
  if (fabs(speed_) < 0.02) 
  {
    speed_direction = 0;
  } 
  else if (speed_ < 0) 
  {
    speed_direction = 2;
  } 
  else 
  {
    speed_direction = 1;
  }
  uint32_t speed_value = static_cast<uint32_t>(fabs(speed_) / 0.02);
  Byte frame_speed_direction(data);
  frame_speed_direction.set_value(
      static_cast<unsigned char>((speed_direction << 6) & 0x00C0) |
          static_cast<unsigned char>((speed_value & 0x1F00) >> 8),
      0, 8);
  Byte frame_speed(data + 1);
  frame_speed.set_value(static_cast<unsigned char>(speed_value & 0x00FF), 0, 8);
}


void MotionInputSpeed300::Reset() { speed_ = NAN; }

void MotionInputSpeed300::SetSpeed(const float& speed) { speed_ = speed; }



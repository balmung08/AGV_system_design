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

#include "config/motion_input_yawrate_301.h"

#include "Byte/byte.h"



const uint32_t MotionInputYawRate301::ID = 0x301;

MotionInputYawRate301::MotionInputYawRate301() {}
MotionInputYawRate301::~MotionInputYawRate301() {}

uint32_t MotionInputYawRate301::GetPeriod() const {
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

/**
 * @brief update the data
 * @param data a pointer to the data to be updated
 */
void MotionInputYawRate301::UpdateData(uint8_t* data) {
  if (std::isnan(yaw_rate_)) {
    AWARN << "yaw_rate is nan";
    return;
  }
  // Due to radar 408 manual: max 327.68, res 0.01, unit:deg/s
  uint32_t yaw_rate_value = static_cast<uint32_t>((yaw_rate_ + 327.68) * 100);
  Byte yaw_rate_low(data);
  yaw_rate_low.set_value(
      static_cast<unsigned char>((yaw_rate_value & 0xFF00) >> 8), 0, 8);
  Byte yaw_rate_high(data + 1);
  yaw_rate_high.set_value(static_cast<unsigned char>(yaw_rate_value & 0x00FF),
                          0, 8);
}

/**
 * @brief reset the private variables
 */
void MotionInputYawRate301::Reset() { yaw_rate_ = NAN; }

void MotionInputYawRate301::SetYawRate(const float& yaw_rate) {
  yaw_rate_ = yaw_rate;
}


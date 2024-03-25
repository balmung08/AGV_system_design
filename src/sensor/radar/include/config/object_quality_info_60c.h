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
#include "radar/configuration_vars.h"
#include "radar/const_vars.h"

class ObjectQualityInfo60C
{
 public:
  static const uint32_t ID;
  ObjectQualityInfo60C();
  
  int object_id(const uint8_t* bytes, int32_t length) const;

  int longitude_dist_rms(const uint8_t* bytes, int32_t length) const;

  int lateral_dist_rms(const uint8_t* bytes, int32_t length) const;

  int longitude_vel_rms(const uint8_t* bytes, int32_t length) const;

  int lateral_vel_rms(const uint8_t* bytes, int32_t length) const;

  int longitude_accel_rms(const uint8_t* bytes, int32_t length) const;

  int lateral_accel_rms(const uint8_t* bytes, int32_t length) const;

  int oritation_angle_rms(const uint8_t* bytes, int32_t length) const;

  int probexist(const uint8_t* bytes, int32_t length) const;

  int meas_state(const uint8_t* bytes, int32_t length) const;
};


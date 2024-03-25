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

#include "config/object_quality_info_60c.h"
#include "Byte/byte.h"



ObjectQualityInfo60C::ObjectQualityInfo60C() {}
const uint32_t ObjectQualityInfo60C::ID = 0x60C;


int ObjectQualityInfo60C::object_id(const uint8_t* bytes,
                                    int32_t length) const {
  Byte t0(bytes);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

int ObjectQualityInfo60C::longitude_dist_rms(const uint8_t* bytes,
                                             int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(3, 5);

  int ret = x;
  return ret;
}

int ObjectQualityInfo60C::lateral_dist_rms(const uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 3);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(6, 2);

  x <<= 2;
  x |= t;

  int ret = x;
  return ret;
}

int ObjectQualityInfo60C::longitude_vel_rms(const uint8_t* bytes,
                                            int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(1, 5);

  int ret = x;
  return ret;
}

int ObjectQualityInfo60C::lateral_vel_rms(const uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 1);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(4, 4);

  x <<= 4;
  x |= t;

  int ret = x;
  return ret;
}

int ObjectQualityInfo60C::longitude_accel_rms(const uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 4);

  Byte t1(bytes + 4);
  int32_t t = t1.get_byte(7, 1);

  x <<= 1;
  x |= t;

  int ret = x;
  return ret;
}

int ObjectQualityInfo60C::lateral_accel_rms(const uint8_t* bytes,
                                            int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(2, 5);

  int ret = x;
  return ret;
}

int ObjectQualityInfo60C::oritation_angle_rms(const uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 2);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(5, 3);

  x <<= 3;
  x |= t;

  int ret = x;
  return ret;
}

int ObjectQualityInfo60C::probexist(const uint8_t* bytes,
                                    int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(5, 3);

  int ret = x;
  return ret;
}

int ObjectQualityInfo60C::meas_state(const uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(2, 3);

  int ret = x;
  return ret;
}


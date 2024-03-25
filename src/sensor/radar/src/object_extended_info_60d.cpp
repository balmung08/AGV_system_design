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

#include "config/object_extended_info_60d.h"
#include "Byte/byte.h"


ObjectExtendedInfo60D::ObjectExtendedInfo60D() {}
const uint32_t ObjectExtendedInfo60D::ID = 0x60D;


int ObjectExtendedInfo60D::object_id(const uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

double ObjectExtendedInfo60D::longitude_accel(const uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(5, 3);

  x <<= 3;
  x |= t;

  double ret = x * OBJECT_AREL_RES + OBJECT_AREL_LONG_MIN;
  return ret;
}

double ObjectExtendedInfo60D::lateral_accel(const uint8_t* bytes,
                                            int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 5);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(4, 4);

  x <<= 4;
  x |= t;

  double ret = x * OBJECT_AREL_RES + OBJECT_AREL_LAT_MIN;
  return ret;
}

int ObjectExtendedInfo60D::obstacle_class(const uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 3);

  int ret = x;
  return ret;
}

double ObjectExtendedInfo60D::oritation_angle(const uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(6, 2);

  x <<= 2;
  x |= t;

  double ret = x * OBJECT_ORIENTATION_ANGEL_RES + OBJECT_ORIENTATION_ANGEL_MIN;
  return ret;
}

double ObjectExtendedInfo60D::object_length(const uint8_t* bytes,
                                            int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * OBJECT_LENGTH_RES;
  return ret;
}

double ObjectExtendedInfo60D::object_width(const uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * OBJECT_WIDTH_RES;
  return ret;
}

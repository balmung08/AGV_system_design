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

#include "config/object_general_info_60b.h"
#include "Byte/byte.h"


ObjectGeneralInfo60B::ObjectGeneralInfo60B() {}
const uint32_t ObjectGeneralInfo60B::ID = 0x60B;


int ObjectGeneralInfo60B::object_id(const uint8_t* bytes,
                                    int32_t length) const {
  Byte t0(bytes);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

double ObjectGeneralInfo60B::longitude_dist(const uint8_t* bytes,
                                            int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(3, 5);

  x <<= 5;
  x |= t;

  double ret = x * OBJECT_DIST_RES + OBJECT_DIST_LONG_MIN;
  return ret;
}

double ObjectGeneralInfo60B::lateral_dist(const uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 3);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);

  x <<= 8;
  x |= t;

  double ret = x * OBJECT_DIST_RES + OBJECT_DIST_LAT_MIN;
  return ret;
}

double ObjectGeneralInfo60B::longitude_vel(const uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);
  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(6, 2);

  x <<= 2;
  x |= t;
  double ret = x * OBJECT_VREL_RES + OBJECT_VREL_LONG_MIN;
  return ret;
}

double ObjectGeneralInfo60B::lateral_vel(const uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 6);

  Byte t1(bytes + 6);
  int32_t t = t1.get_byte(5, 3);

  x <<= 3;
  x |= t;

  double ret = x * OBJECT_VREL_RES + OBJECT_VREL_LAT_MIN;
  return ret;
}

double ObjectGeneralInfo60B::rcs(const uint8_t* bytes,
                                 int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * OBJECT_RCS_RES + OBJECT_RCS_MIN;
  return ret;
}

int ObjectGeneralInfo60B::dynprop(const uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 3);

  int ret = x;
  return ret;
}


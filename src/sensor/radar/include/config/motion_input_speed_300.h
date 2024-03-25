/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <cmath>
#include <stdint.h>

class MotionInputSpeed300
 {
 public:
  static const uint32_t ID;
  MotionInputSpeed300();
  ~MotionInputSpeed300();
  /**
   * @brief get the data period
   * @return the value of data period
   */
  uint32_t GetPeriod();

  /**
   * @brief update the data
   * @param data a pointer to the data to be updated
   */
  void UpdateData(uint8_t* data);

  /**
   * @brief reset the private variables
   */
  void Reset() ;

  void SetSpeed(const float& speed);
  
  float speed_ = NAN;
};

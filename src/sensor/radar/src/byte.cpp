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

#include "Byte/byte.h"

#include <algorithm>
#include <bitset>



namespace {

    constexpr int32_t BYTE_LENGTH = static_cast<int32_t>(sizeof(int8_t) * 8);

    const uint8_t RANG_MASK_1_L[] = {0x01, 0x03, 0x07, 0x0F,
                                     0x1F, 0x3F, 0x7F, 0xFF};
    const uint8_t RANG_MASK_0_L[] = {0xFE, 0XFC, 0xF8, 0xF0,
                                     0xE0, 0xC0, 0x80, 0x00};

}  // namespace

Byte::Byte(const uint8_t *value) : value_(const_cast<uint8_t *>(value)) {}

Byte::Byte(const Byte &value) : value_(value.value_) {}

std::string Byte::byte_to_hex(const uint8_t value) {
    static const char HEX[] = "0123456789ABCDEF";
    uint8_t high = static_cast<uint8_t>(value >> 4);
    uint8_t low = static_cast<uint8_t>(value & 0x0F);
    return {HEX[high], HEX[low]};
}

std::string Byte::byte_to_hex(const uint32_t value) {
    uint8_t high;
    uint8_t low;
    std::string result = "";
    high = static_cast<uint8_t>((value >> 8) & 0xFF);
    low = static_cast<uint8_t>(value & 0xFF);
    result += byte_to_hex(high);
    result += byte_to_hex(low);
    return result;
}

std::string Byte::byte_to_binary(const uint8_t value) {
    return std::bitset<8 * sizeof(uint8_t)>(value).to_string();
}

void Byte::set_bit_1(const int32_t pos) {
    static const uint8_t BIT_MASK_1[] = {0x01, 0x02, 0x04, 0x08,
                                         0x10, 0x20, 0x40, 0x80};
    if (pos >= 0 && pos < BYTE_LENGTH) {
        *value_ |= BIT_MASK_1[pos];
    }
}

void Byte::set_bit_0(const int32_t pos) {
    static const uint8_t BIT_MASK_0[] = {0xFE, 0xFD, 0xFB, 0xF7,
                                         0xEF, 0xDF, 0xBF, 0x7F};
    if (pos >= 0 && pos < BYTE_LENGTH) {
        *value_ &= BIT_MASK_0[pos];
    }
}

bool Byte::is_bit_1(const int32_t pos) const {
    return pos >= 0 && pos < BYTE_LENGTH && ((*value_ >> pos) % 2 == 1);
}

void Byte::set_value(const uint8_t value) {
    if (value_ != nullptr) {
        *value_ = value;
    }
}

void Byte::set_value_high_4_bits(const uint8_t value) {
    set_value(value, 4, 4);
}

void Byte::set_value_low_4_bits(const uint8_t value) { set_value(value, 0, 4); }

void Byte::set_value(const uint8_t value, const int32_t start_pos,
                     const int32_t length) {
    if (start_pos > BYTE_LENGTH - 1 || start_pos < 0 || length < 1) {
        return;
    }
    int32_t end_pos = std::min(start_pos + length - 1, BYTE_LENGTH - 1);
    int32_t real_len = end_pos + 1 - start_pos;
    uint8_t current_value_low = 0x00;
    if (start_pos > 0) {
        current_value_low = *value_ & RANG_MASK_1_L[start_pos - 1];
    }
    uint8_t current_value_high = *value_ & RANG_MASK_0_L[end_pos];
    uint8_t middle_value = value & RANG_MASK_1_L[real_len - 1];
    middle_value = static_cast<uint8_t>(middle_value << start_pos);
    *value_ = static_cast<uint8_t>(current_value_high + middle_value +
                                   current_value_low);
}

uint8_t Byte::get_byte() const { return *value_; }

uint8_t Byte::get_byte_high_4_bits() const { return get_byte(4, 4); }

uint8_t Byte::get_byte_low_4_bits() const { return get_byte(0, 4); }

uint8_t Byte::get_byte(const int32_t start_pos, const int32_t length) const {
    if (start_pos > BYTE_LENGTH - 1 || start_pos < 0 || length < 1) {
        return 0x00;
    }
    int32_t end_pos = std::min(start_pos + length - 1, BYTE_LENGTH - 1);
    int32_t real_len = end_pos + 1 - start_pos;
    uint8_t result = static_cast<uint8_t>(*value_ >> start_pos);
    result &= RANG_MASK_1_L[real_len - 1];
    return result;
}

std::string Byte::to_hex_string() const { return byte_to_hex(*value_); }

std::string Byte::to_binary_string() const { return byte_to_binary(*value_); }


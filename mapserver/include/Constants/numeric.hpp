#ifndef CONSTANTS_NUMERIC_HPP_
#define CONSTANTS_NUMERIC_HPP_

#include "Constants.hpp"

namespace constants {
namespace numeric {
const int8_t invalidValue_int8 = 0x80;
const int16_t invalidValue_int16 = 0x8000;
const int32_t invalidValue_int32 = 0x80000000;
const int64_t invalidValue_int64 = 0x8000000000000000;
const float invalidValue_float = -FLT_MAX;
const double invalidValue_double = -DBL_MAX;
const int8_t invalidValueP_int8 = 0x7F;
const int16_t invalidValueP_int16 = 0x7FFF;
const int32_t invalidValueP_int32 = 0x7FFFFFFF;
const int64_t invalidValueP_int64 = 0x7FFFFFFFFFFFFFFF;
const float invalidValueP_float = FLT_MAX;
const double invalidValueP_double = DBL_MAX;
}
}
#endif // CONSTANTS_NUMERIC_HPP_

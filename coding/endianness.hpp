#pragma once

#include "base/base.hpp"

#include "std/type_traits.hpp"

#include <cstddef>

// @TODO(bykoianko) This method returns false since 05.12.2010. That means only little endian
// architecture is supported. Now two checks are added that only little endian architecture
// supported. Consider remove all code that uses IsBigEndian() method except for checks.
inline bool IsBigEndian()
{
  uint16_t const word = 0x0001;
  char const * const b = reinterpret_cast<char const * const>(&word);
  return b[0] == 0x0;
}

inline bool IsLittleEndian()
{
  return !IsBigEndian();
}

template <typename T> T ReverseByteOrder(T t)
{
  static_assert(is_integral<T>::value, "Only integral types are supported.");

  T res;
  char const * a = reinterpret_cast<char const *>(&t);
  char * b = reinterpret_cast<char *>(&res);
  for (size_t i = 0; i < sizeof(T); ++i)
    b[i] = a[sizeof(T) - 1 - i];
  return res;
}

template <typename T> inline T SwapIfBigEndian(T t)
{
  if (IsBigEndian())
    return ReverseByteOrder(t);

  return t;
}

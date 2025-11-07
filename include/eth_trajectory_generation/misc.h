#ifndef TRAJECTORY_GENERATION_MISC_H
#define TRAJECTORY_GENERATION_MISC_H

#include <iostream>

#define CHECK_LT(v1, v2)                                                                                                                                       \
  if (!(v1 < v2))                                                                                                                                              \
    std::cout << __FILE__ << ":" << __LINE__ << " Error: " << v1 << " >= " << v2 << std::endl;

#define CHECK_GT(v1, v2)                                                                                                                                       \
  if (!(v1 > v2))                                                                                                                                              \
  std::cout << __FILE__ << ":" << __LINE__ << " Error: " << v1 << " <= " << v2

#define CHECK_GE(v1, v2)                                                                                                                                       \
  if (!(v1 >= v2))                                                                                                                                             \
  std::cout << __FILE__ << ":" << __LINE__ << " Error: " << v1 << " < " << v2

#define CHECK_LE(v1, v2)                                                                                                                                       \
  if (!(v1 <= v2))                                                                                                                                             \
    std::cout << __FILE__ << ":" << __LINE__ << " Error: " << v1 << " > " << v2 << std::endl;

#define CHECK_EQ(v1, v2)                                                                                                                                       \
  if (!(v1 == v2))                                                                                                                                             \
  std::cout << __FILE__ << ":" << __LINE__ << " Error: " << v1 << " != " << v2

#define CHECK_NOTNULL(v)                                                                                                                                       \
  if ((v == NULL))                                                                                                                                             \
    std::cout << __FILE__ << ":" << __LINE__ << " Error: " << v << " == NULL" << std::endl;

#define CHECK(v)                                                                                                                                               \
  if (!(v))                                                                                                                                                    \
  std::cout << __FILE__ << ":" << __LINE__ << " Error: << !(v) "

#define LOG(v) std::cout

#define DLOG(v) std::cout

#define VLOG(v) std::cout

#endif // TRAJECTORY_GENERATION_MISC_H

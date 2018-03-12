#pragma once

#include <cmath>
#include <vector>
#include <stdexcept>
#include <iostream>

using namespace std;


#define DISALLOW_COPY(className) \
     className(const className &) = delete; \
     className &operator=(const className &) = delete;


template <typename T>
inline const T &mMin(const T &a, const T &b) { if (a < b) return a; return b; }
template <typename T>
inline const T &mMax(const T &a, const T &b) { if (a < b) return b; return a; }

inline double mRound(double x) {
    return floor(x + 0.5);
}



#ifdef _DEBUG
#define M_ASSERT(cond) do { if (!(cond)) throw std::runtime_error(#cond); } while(0)
#else
#define M_ASSERT(cond)
#endif

#define M_CHECK(cond) do { if (!(cond)) throw std::runtime_error(#cond); } while(0)

#define LOG(strm) do { std::cout << strm << std::endl; } while(0)

#define LOG_ERR(strm) do { std::cout << "ERROR: " << strm << std::endl; } while(0)

#ifndef THREADING_H
#define THREADING_H

#ifdef __cplusplus
#include <vector>
#include <cstddef>
#endif

extern std::vector<float> GRF, Angle, GRFTime, AngleTime;

#ifdef __cplusplus
extern "C" {
#endif

const float* get_GRF_data();
size_t get_GRF_size();
const float* get_GRFTime_data();
size_t get_GRFTime_size();

#ifdef __cplusplus
}
#endif

#endif // THREADING_H
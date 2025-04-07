#ifndef GRF_H
#define GRF_H

#include <vector>
#include <cmath>

// Function declarations
std::vector<float> calculate_length(std::vector<float> angle, float length);
std::vector<float> entrywise_mul(std::vector<float> L, float COM);
std::vector<float> entrywise_add(std::vector<float> vect_A, std::vector<float> vect_B);
std::vector<float> crossProduct(std::vector<float> vect_A, std::vector<float> vect_B);
std::vector<float> integrate(std::vector<float> a, std::vector<float> b, float dt);
std::vector<float> differentiate(std::vector<float> a, std::vector<float> pre_a, std::vector<float> b, float dt);
std::vector<float> calculate_grf(std::vector<float> a_shank, std::vector<float> a_thigh, std::vector<float> a_hip, float m);

#endif // GRF_H
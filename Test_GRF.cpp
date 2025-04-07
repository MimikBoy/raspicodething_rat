#include <iostream>
#define _USE_MATH_DEFINES // Enable M_PI and other math constants
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "GRF.h"

void test_calculate_length() {
    std::vector<float> angle = {M_PI / 4, 0.0f, M_PI / 4}; // 45 degrees
    float length = 1.0f;
    std::vector<float> result = calculate_length(angle, length);
    std::cout << "Test calculate_length: ";
    std::cout << "Expected: {0.707, 0.0, 0.707}, Got: {" 
              << result[0] << ", " << result[1] << ", " << result[2] << "}" << std::endl;
}

void test_entrywise_mul() {
    std::vector<float> L = {1.0f, 2.0f, 3.0f};
    float COM = 0.5f;
    std::vector<float> result = entrywise_mul(L, COM);
    std::cout << "Test entrywise_mul: ";
    std::cout << "Expected: {0.5, 1.0, 1.5}, Got: {" 
              << result[0] << ", " << result[1] << ", " << result[2] << "}" << std::endl;
}

void test_entrywise_add() {
    std::vector<float> vect_A = {1.0f, 2.0f, 3.0f};
    std::vector<float> vect_B = {4.0f, 5.0f, 6.0f};
    std::vector<float> result = entrywise_add(vect_A, vect_B);
    std::cout << "Test entrywise_add: ";
    std::cout << "Expected: {5.0, 7.0, 9.0}, Got: {" 
              << result[0] << ", " << result[1] << ", " << result[2] << "}" << std::endl;
}

int main() {
    test_calculate_length();
    test_entrywise_mul();
    test_entrywise_add();
    return 0;
}
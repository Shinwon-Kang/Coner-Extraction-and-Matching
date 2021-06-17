#ifndef VECTOR_UTILS_HEADER_
#define VECTOR_UTILS_HEADER_

#include "vector"
#include "VectorUtils.hpp"
#include "cmath"
#include "iostream"

namespace VectorUtils {
    float GetTwoPointsDistance(std::vector<float> v_from, std::vector<float> v_to);
    float GetTwoPointsAngle(std::vector<float> v_from, std::vector<float> v_to);
    float GetTwoPointsGradient(std::vector<float> v_from, std::vector<float> v_to);
    std::vector<float> GetEquationParameter(std::vector<float> v_from, std::vector<float> v_to);
    float GetPointToEquationDistance(std::vector<float> equation, std::vector<float> point);
}

#endif
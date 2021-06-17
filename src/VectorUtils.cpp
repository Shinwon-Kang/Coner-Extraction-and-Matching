#include "VectorUtils.hpp"

namespace VectorUtils
{
    float GetTwoPointsDistance(std::vector<float> v_from, std::vector<float> v_to) {
        return sqrt(pow((v_from[0] - v_to[0]), 2) + pow((v_from[1] - v_to[1]), 2));
    }

    float GetTwoPointsAngle(std::vector<float> v_from, std::vector<float> v_to) {
        return atan2(v_to[1] - v_from[1], v_to[0] - v_from[0]) * 180 / M_PI;
    }

    float GetTwoPointsGradient(std::vector<float> v_from, std::vector<float> v_to) {
        return (v_to[1] - v_from[1]) / (v_to[0] - v_from[0]);
    }

    std::vector<float> GetEquationParameter(std::vector<float> v_from, std::vector<float> v_to) {
        std::vector<float> equation_parameter;
        if(v_from[0] == v_to[0]) {
            equation_parameter.push_back(1);
            equation_parameter.push_back(0);
            equation_parameter.push_back(-v_from[0]);
        } else {
            float gradient = GetTwoPointsGradient(v_from, v_to);
            float a = -gradient;
            float b = 1;
            float c = gradient * v_from[0] - v_from[1];

            equation_parameter.push_back(a);
            equation_parameter.push_back(b);
            equation_parameter.push_back(c);
        }

        return equation_parameter;
    }

    float GetPointToEquationDistance(std::vector<float> equation, std::vector<float> point) {
        return fabsf(equation[0] * point[0] + equation[1] * point[1] + equation[2]) / sqrt(pow(equation[0], 2) + pow(equation[1], 2));
    }
}
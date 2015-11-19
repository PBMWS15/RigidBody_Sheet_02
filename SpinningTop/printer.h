#pragma once
#include <glm/vec3.hpp>
#include <iostream>

void printVec(const glm::dvec3& vec)
{
    std::cout << "(" << vec.x << "," << vec.y << "," << vec.z << ")";
}


void printMat(const glm::dmat3& mat)
{
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j)
        {
            std::cout << mat[i][j] << ",";
        }
        std::cout << std::endl;
    }
}
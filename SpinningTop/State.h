#pragma once
#include <glm/glm.hpp>
class State
{
public:
    State(const glm::dvec3 pos, const glm::dvec3& vel);
    ~State();

    glm::dvec3 getPosition();
    glm::dvec3 getVelocity();

    void setPosition(const glm::dvec3& pos);
    void setVelocity(const glm::dvec3& vel);
private:
    glm::dvec3 _position;
    glm::dvec3 _velocity;

};


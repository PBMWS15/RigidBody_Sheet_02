#include "State.h"


State::State(const glm::dvec3 pos, const glm::dvec3& vel): _position{pos},_velocity{vel}
{
}

State::~State()
{
}

glm::dvec3 State::getPosition()
{
    return _position;
}

glm::dvec3 State::getVelocity()
{
    return _velocity;
}

void State::setPosition(const glm::dvec3& pos)
{
    _position = pos;
}

void State::setVelocity(const glm::dvec3& vel)
{
    _velocity = vel;
}
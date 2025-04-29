#include "BGTNode.h"

BGTNode::BGTNode(
    const unsigned int distance,
    const unsigned int heightSpeed)
        : m_distance(distance),
        m_heightSpeed(heightSpeed)
{
}

unsigned int BGTNode::GetDistance() const
{
    return m_distance;
}

unsigned int BGTNode::GetHeightSpeed() const
{
    return m_heightSpeed;
}

unsigned int BGTNode::GetHeight() const
{
    return m_height;
}

void BGTNode::IncreaseHeight(const float growCoefficient)
{
    m_height += m_heightSpeed * growCoefficient;
}

void BGTNode::Cut()
{
    m_height = 0;
}
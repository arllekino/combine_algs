#pragma once

class BGTNode
{
public:
    BGTNode(unsigned int distance, unsigned int heightSpeed);

    unsigned int GetDistance() const;
    unsigned int GetHeightSpeed() const;
    unsigned int GetHeight() const;
    void IncreaseHeight(float growCoefficient = 1);
    void Cut();

private:
    const unsigned int m_distance;
    const unsigned int m_heightSpeed;
    unsigned int m_height = 0;
};

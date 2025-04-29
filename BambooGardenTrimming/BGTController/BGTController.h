#pragma once

#include "../BGT/BambooGardenTrimming.h"
#include "../BGTNode/BGTNode.h"

class BGTController
{
public:
    BGTController(const std::unordered_set<BGTNode>& nodes);
    void OnInput();
private:
    BambooGardenTrimming m_bambooGardenTrimming;
};
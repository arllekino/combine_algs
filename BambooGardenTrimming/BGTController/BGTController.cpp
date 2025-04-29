#include "BGTController.h"

BGTController::BGTController(const std::unordered_set<BGTNode>& nodes) : m_bambooGardenTrimming(nodes)
{
}

void BGTController::OnInput()
{
    m_bambooGardenTrimming.Update();
}

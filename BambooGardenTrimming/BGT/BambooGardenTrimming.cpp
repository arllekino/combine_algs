#include "BambooGardenTrimming.h"

BambooGardenTrimming::BambooGardenTrimming(const std::unordered_set<BGTNode>& nodes)
    : m_lowerLimit(DefineLowerLimit(nodes)),
    m_upperLimit(DefineUpperLimit(nodes)),
    m_nodes(ConvertNodesToPtrNodes(nodes))
{
}

void BambooGardenTrimming::Update()
{
    UpdateNodes();

    if (m_servicedNode == nullptr && !m_requestedNodes.empty())
    {
        m_servicedNode = GetNodeWithClosestDeadline();
    }

    if (m_servicedNode != nullptr)
    {
        UpdateServicedNode();
    }
}

std::unordered_set<std::shared_ptr<BGTNode>> BambooGardenTrimming::ConvertNodesToPtrNodes(const std::unordered_set<BGTNode>& nodes)
{
    std::unordered_set<std::shared_ptr<BGTNode>> result;
    // std::transform(nodes.begin(), no)

    for (const auto& node : nodes)
    {
        result.insert(std::make_shared<BGTNode>(node));
    }
    return result;
}

double BambooGardenTrimming::DefineLCoefficient(const std::unordered_set<BGTNode>& nodes)
{
    const auto maxSpeed = std::ranges::max_element(nodes.begin(), nodes.end(),
        [](const BGTNode& a, const BGTNode& b) {
            return a.GetHeightSpeed() < b.GetHeightSpeed();
        });
    const auto maxDistance = std::ranges::max_element(nodes.begin(), nodes.end(),
        [](const BGTNode& a, const BGTNode& b) {
            return a.GetDistance() < b.GetDistance();
        });

    unsigned int summaryGrow = 0;
    for (const auto& node : nodes)
    {
        summaryGrow += node.GetHeightSpeed() * node.GetDistance();
    }

    return std::max(maxSpeed * maxDistance, summaryGrow);
}

double BambooGardenTrimming::DefineLowerLimit(const std::unordered_set<BGTNode>& nodes)
{
    return (1 + sqrt(2)) * DefineLCoefficient(nodes);
}

double BambooGardenTrimming::DefineUpperLimit(const std::unordered_set<BGTNode>& nodes)
{
    return (3 + 2 * sqrt(2)) * DefineLCoefficient(nodes);
}

void BambooGardenTrimming::UpdateNodes()
{
    for (const auto& node : m_nodes)
    {
        node->IncreaseHeight();
        if (node->GetHeightSpeed() >= m_lowerLimit)
        {
            m_requestedNodes.insert(node);
        }
    }
}

void BambooGardenTrimming::UpdateServicedNode()
{
    if (!isNodeCut)
    {
        if (progressToNode + 1 >= m_servicedNode->GetDistance() / 2)
        {
            m_servicedNode->Cut();
            isNodeCut = true;
            const auto heightCoefAfterCutting = progressToNode + 1 - m_servicedNode->GetDistance() / 2;
            m_servicedNode->IncreaseHeight(heightCoefAfterCutting);
        }
        progressToNode++;
    }
    else
    {
        if (progressToNode + 1 >= m_servicedNode->GetDistance())
        {
            m_servicedNode = nullptr;
            progressToNode = 0;
            isNodeCut = false;
        }
        else
        {
            progressToNode++;
        }
    }
}

std::shared_ptr<BGTNode> BambooGardenTrimming::GetNodeWithClosestDeadline()
{
    return *std::ranges::max_element(m_requestedNodes.begin(), m_requestedNodes.end(),
        [](const auto& a, const auto& b) {
            return a->GetHeight() < b->GetHeight();
        });
}

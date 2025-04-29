#pragma once

#include "../BGTNode/BGTNode.h"

#include <unordered_set>

class BambooGardenTrimming
{
public:
    explicit BambooGardenTrimming(const std::unordered_set<BGTNode>& nodes);

    void Update();

private:
    const double m_lowerLimit;
    const double m_upperLimit;

    std::unordered_set<std::shared_ptr<BGTNode>> m_nodes;
    std::unordered_set<std::shared_ptr<BGTNode>> m_requestedNodes;
    std::shared_ptr<BGTNode> m_servicedNode;

    int progressToNode = 0;
    bool isNodeCut = false;

    static std::unordered_set<std::shared_ptr<BGTNode>> ConvertNodesToPtrNodes(const std::unordered_set<BGTNode>& nodes);
    static double DefineLCoefficient(const std::unordered_set<BGTNode>& nodes);
    static double DefineLowerLimit(const std::unordered_set<BGTNode>& nodes);
    static double DefineUpperLimit(const std::unordered_set<BGTNode>& nodes);

    void UpdateNodes();
    void UpdateServicedNode();
    std::shared_ptr<BGTNode> GetNodeWithClosestDeadline();
};
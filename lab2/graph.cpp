#include "graph.h"

#include <fstream>
#include <iostream>

Graph::Graph(int nodeCount, const std::vector<GraphEdge>& edges)
{
    this->InitMatrixSize(nodeCount);
    this->CreateGraphByEdgeList(edges);
}

Graph::~Graph()
{
}

bool Graph::IsError()
{
    return this->isError;
}

void Graph::CreateGraphByEdgeList(const std::vector<GraphEdge>& edges)
{
    for (const auto &edge : edges)
    {
        if (this->isError)
        {
            return;
        }
        this->AddPointToMatrix(edge);
    }
    this->CreateGraphInNode();
}

void Graph::PrintAdjacencyMatrix()
{
    for (const auto& row : this->adjacencyMatrix)
    {
        for (const auto& node : row)
        {
            if (node)
            {
                std::cout << node << " ";
            }
            else
            {
                std::cout << "0 ";
            }
        }
        std::cout << std::endl;
    }
}

void Graph::AddPointToMatrix(const GraphEdge& params)
{   
    const auto& matrixSize = this->adjacencyMatrix.size();
    if (params.from > matrixSize - 1 || params.to > matrixSize - 1)
    {
        this->isError = true;
        return;
    }

    this->adjacencyMatrix[params.from][params.to] = params.howMuch;
    this->adjacencyMatrix[params.to][params.from] = params.howMuch;
}

void Graph::CreateGraphInNode()
{
    this->nodes.clear();

    for (size_t i = 0; i < this->adjacencyMatrix.size(); i++)
    {
        auto node = std::make_shared<Node>(i);
        this->nodes.push_back(node);
    }

    for (size_t i = 0; i < this->adjacencyMatrix.size(); i++)
    {
        for (size_t j = 0; j < this->adjacencyMatrix[i].size(); j++)
        {
            if (this->adjacencyMatrix[i][j] != 0)
            {
                this->nodes[i]->nextPtrs.push_back({j, this->nodes[j]});
                this->nodes[i]->nextPtrs.push_back({j, this->nodes[j]});
            }
        }
    }
}

void Graph::InitMatrixBeforeDFS(std::stack<std::pair<std::shared_ptr<Node>, bool>>& stack)
{
    for (auto &node : nodes)
    {
        node->visited = false;
    }

    for (auto &node : nodes)
    {
        if (!node->visited)
        {
            stack.push({node, false});
        }
    }
}

void Graph::DFS()
{
    std::stack<std::pair<std::shared_ptr<Node>, bool>> stack;
    this->InitMatrixBeforeDFS(stack);

    std::cout << "Start index: " << stack.top().first->index << std::endl;
    int i = 0;
    while (!stack.empty())
    {
        auto [current, processed] = stack.top();
        stack.pop();

        if (!processed)
        {
            if (current->visited) continue;
            current->visited = true;
            current->entryTime = i++;
            stack.push({current, true});

            for (auto it = current->nextPtrs.rbegin(); it != current->nextPtrs.rend(); ++it)
            {
                auto neighbor = it->second;
                if (!neighbor->visited)
                {
                    stack.push({neighbor, false});
                }
            }
        }
        else
        {
            current->exitTime = i++;
            std::cout << "Node " << current->index <<
                " [" << current->entryTime << ", " << current->exitTime << "]" << std::endl;
        }
    }
}

void Graph::InitMatrixSize(int size)
{
    adjacencyMatrix.resize(size, std::vector<int>(size, 0));
    for (auto &row : this->adjacencyMatrix)
    {
        row.resize(size, 0);
    }
}

void Graph::DFSUtil(std::shared_ptr<Node> node, int parentIndex, int& time, std::vector<bool>& isArticulationPoint)
{
    node->visited = true;
    node->entryTime = node->low = time++;
    int children = 0;

    for (auto& [index, neighbor] : node->nextPtrs)
    {
        if (index == parentIndex) continue;

        if (neighbor->visited)
        {
            node->low = std::min(node->low, neighbor->entryTime);
        }
        else
        {
            DFSUtil(neighbor, node->index, time, isArticulationPoint);
            node->low = std::min(node->low, neighbor->low);

            if (neighbor->low >= node->entryTime && parentIndex != -1)
            {
                isArticulationPoint[node->index] = true;
            }
            children++;
        }
    }

    if (parentIndex == -1 && children > 1)
    {
        isArticulationPoint[node->index] = true;
    }
}

std::vector<size_t> Graph::FindArticulationPoint()
{
    if (nodes.empty()) return std::vector<size_t>();

    std::vector<bool> isArticulationPoint(nodes.size(), false);
    int time = 0;

    for (auto& node : nodes)
    {
        if (!node->visited)
        {
            DFSUtil(node, -1, time, isArticulationPoint);
        }
    }

    std::vector<size_t> articulationPoints;
    for (size_t i = 0; i < isArticulationPoint.size(); ++i)
    {
        if (isArticulationPoint[i])
        {
            articulationPoints.push_back(i);
        }
    }
    return articulationPoints;
}

void Graph::PrintPoints(const std::vector<size_t>& points)
{
    for (const auto& p : points) {
        std::cout << p << " ";
    }
    std::cout << std::endl;
}
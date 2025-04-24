#pragma once

#include <vector>
#include <sstream>

struct Node
{
    int index;
    std::shared_ptr<Node> parent = nullptr;
    std::vector<std::pair<int, std::shared_ptr<Node>>> nextPtrs;
    bool visited = false;

    int entryTime = -1;
    int exitTime = - 1;
    int low = -1;

    Node(int index) : index(index) {}
};

struct GraphEdge
{
    size_t from;
    size_t to;
    int howMuch;

    GraphEdge(
        size_t from,
        size_t to,
        int howMuch
    ) : from(from), to(to), howMuch(howMuch)
    {}
};

using Matrix = std::vector<std::vector<int>>;

class Graph
{
private:
    Matrix adjacencyMatrix;
    bool isError = false;
    std::vector<std::shared_ptr<Node>> nodes;

    void AddPointToMatrix(const GraphEdge& params);
    void CreateGraphInNode();
    void InitMatrixSize(int size);
    void InitMatrixBeforeDFS(std::stack<std::pair<std::shared_ptr<Node>, bool>>& stack);
    void DFSUtil(std::shared_ptr<Node> node, int parentIndex, int& time, std::vector<bool>& isArticulationPoint);

public:
    Graph(int nodeCount, const std::vector<GraphEdge>& edges);
    ~Graph();
    void CreateGraphByEdgeList(const std::vector<GraphEdge>& edges);
    void PrintAdjacencyMatrix();
    void DFS();
    bool IsError();
    std::vector<size_t> FindArticulationPoint();
    void PrintPoints(const std::vector<size_t>& points);
};
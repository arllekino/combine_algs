#include "graph.h"

#include <iostream>
#include <fstream>

struct GraphParams
{
    int nodeCount;
    std::vector<GraphEdge> edges;

    GraphParams(int count, std::vector<GraphEdge> edges) : nodeCount(count), edges(edges) {}
};

GraphParams LoadGraphFromFile(const std::string &filePath);
GraphEdge ParseParamsFromSString(std::stringstream &sLine);

GraphParams LoadGraphFromFile(const std::string &filePath)
{
    std::vector<GraphEdge> edges;

    std::ifstream file(filePath);
    if (!file.is_open())
    {
        std::cout << "Failed to open file" << filePath << std::endl;
        return {0, edges};
    }

    std::string line;

    getline(file, line);

    int count = 0;
    try
    {
        count = std::stoi(line);
    }
    catch(const std::invalid_argument& e)
    {
        std::cerr << e.what() << '\n';
        return {0, edges};
    }
    

    while (getline(file, line))
    {
        std::stringstream sLine(line);
        auto edge = ParseParamsFromSString(sLine);
        edges.push_back(edge);
    }

    return {count, edges};
}

GraphEdge ParseParamsFromSString(std::stringstream &sLine)
{
    std::string firstArg;
    std::string secondArg;
    std::string thirdArg;

    sLine >> firstArg >> secondArg >> thirdArg;
    if (sLine.fail())
    {
        return {0, 0, 0};
    }

    try
    {
        return {std::stoul(firstArg), std::stoul(secondArg), std::stoi(thirdArg)};
    }
    catch (const std::invalid_argument &e)
    {
        std::cerr << e.what() << '\n';
        return {0, 0, 0};
    }
}

int main(int argc, char *argv[])
{
    if (argc == 2)
    {
        auto graphParams = LoadGraphFromFile(argv[1]);
        auto graph = Graph(graphParams.nodeCount, graphParams.edges);
        if (!graph.IsError())
        {
            graph.PrintAdjacencyMatrix();
            auto articulationPoints = graph.FindArticulationPoint();
            graph.PrintPoints(articulationPoints);
        }
    }

    return 0;
}
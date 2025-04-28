#pragma once

#include "../vertex/vertex.h"
#include "../verge/verge.h"

class Edge;

struct EdgeState
{
    std::shared_ptr<Vertex> endPoint;
    std::shared_ptr<Edge> nextEdge; // следующее по часовой стрелке в трегугольнике справа
    std::shared_ptr<Edge> twinEdge; // ребро-близнец, направленное в другую сторону
    std::shared_ptr<Verge> triangle; // указатель на треугольник справа
};

class Edge
{
public:
    Edge(EdgeState state);

    std::shared_ptr<Edge> GetNextEdge() const;
    std::shared_ptr<Edge> GetTwinEdge() const;
    std::shared_ptr<Vertex> GetEndPoint() const;
    std::shared_ptr<Verge> GetTriangle() const;

    void SetNextEdge(std::shared_ptr<Edge> edge);
    void SetTwinEdge(std::shared_ptr<Edge> edge);
    void SetTriangle(std::shared_ptr<Verge> verge);

private:
    EdgeState m_state;
};
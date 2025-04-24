#include "edge.h"

Edge::Edge(EdgeState state)
{
    m_state = state;
}

std::shared_ptr<Edge> Edge::GetNextEdge() const
{
    return m_state.nextEdge;
}

std::shared_ptr<Edge> Edge::GetTwinEdge() const
{
    return m_state.twinEdge;
}

std::shared_ptr<Vertex> Edge::GetEndPoint() const
{
    return m_state.endPoint;
}

std::shared_ptr<Verge> Edge::GetTriangle() const
{
    return m_state.triangle;
}

void Edge::SetNextEdge(std::shared_ptr<Edge> edge) { m_state.nextEdge = edge; }
void Edge::SetTwinEdge(std::shared_ptr<Edge> edge) { m_state.twinEdge = edge; }
void Edge::SetTriangle(std::shared_ptr<Verge> verge) { m_state.triangle = verge; }
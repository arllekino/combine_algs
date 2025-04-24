#include "./vertex.h"

Position Vertex::GetPosition() const
{
    return m_state.position;
}
VertexList Vertex::GetAdjacentVertexList()
{
    return m_state.adjacentVertexList;
}

void Vertex::SetPosition(const Position position)
{
    m_state.position = position;
}

void Vertex::AddAdjacentVertex(const std::shared_ptr<Vertex>& vertex)
{
    m_state.adjacentVertexList.push_back(vertex);
}

void Vertex::DeleteFromAdjacentVertex(const std::shared_ptr<Vertex>& vertex)
{
    m_state.adjacentVertexList.erase(std::ranges::remove(m_state.adjacentVertexList, vertex).begin());
}

bool Vertex::operator==(const Vertex& other) const
{
    return m_state.position.x == other.m_state.position.x &&
           m_state.position.y == other.m_state.position.y;
}
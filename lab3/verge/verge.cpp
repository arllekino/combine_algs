#include "verge.h"

Verge::Verge(
    std::shared_ptr<Vertex> firstVertex,
    std::shared_ptr<Vertex> secondVertex,
    std::shared_ptr<Vertex> thirdVertex,
    std::shared_ptr<Verge> firstVerge = nullptr,
    std::shared_ptr<Verge> secondVerge = nullptr,
    std::shared_ptr<Verge> thirdVerge = nullptr)
{
    SetVertexList(firstVertex, secondVertex, thirdVertex);
    SetTriangles(firstVerge, secondVerge, thirdVerge);
}


VertexList Verge::GetVertexList()
{
    return m_vergeState.vertexList;
}

TriangleList Verge::GetTriangleList()
{
    return  m_vergeState.triangles;
}

void Verge::SetVertexList(std::shared_ptr<Vertex> first, std::shared_ptr<Vertex> second, std::shared_ptr<Vertex> third)
{
    m_vergeState.vertexList.clear();
    m_vergeState.vertexList.emplace_back(first);
    m_vergeState.vertexList.emplace_back(second);
    m_vergeState.vertexList.emplace_back(third);
}

void Verge::SetTriangles(std::shared_ptr<Verge> first, std::shared_ptr<Verge> second, std::shared_ptr<Verge> third)
{
    m_vergeState.triangles.clear();
    m_vergeState.triangles.emplace_back(first);
    m_vergeState.triangles.emplace_back(second);
    m_vergeState.triangles.emplace_back(third);
}

void Verge::SetTriangle(size_t index, std::shared_ptr<Verge> verge)
{
    m_vergeState.triangles[index] = verge;
}
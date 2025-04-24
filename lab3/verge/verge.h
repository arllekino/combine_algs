#pragma once

#include "../vertex/vertex.h"

#include <vector>

class Verge;

using TriangleList = std::vector<std::shared_ptr<Verge>>;

struct VergeState
{
    const int maxSize = 3;
    VertexList vertexList;
    TriangleList triangles;
};

class Verge
{
public:
    Verge(
        std::shared_ptr<Vertex> firstVertex,
        std::shared_ptr<Vertex> secondVertex,
        std::shared_ptr<Vertex> thirdVertex,
        std::shared_ptr<Verge> firstVerge,
        std::shared_ptr<Verge> secondVerge,
        std::shared_ptr<Verge> thirdVerge);

    VertexList GetVertexList();
    TriangleList GetTriangleList();

    void SetVertexList(std::shared_ptr<Vertex> first, std::shared_ptr<Vertex> second, std::shared_ptr<Vertex> third);
    void SetTriangles(std::shared_ptr<Verge> first, std::shared_ptr<Verge> second, std::shared_ptr<Verge> third);
    void SetTriangle(size_t index, std::shared_ptr<Verge> verge);
private:
    VergeState m_vergeState;
};
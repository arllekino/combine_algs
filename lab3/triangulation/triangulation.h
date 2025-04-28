#pragma once
#include "../edge/edge.h"
#include "../verge/verge.h"
#include "../vertex/vertex.h"

#include <set>

class Triangulation {
public:
    void DelaunayTriangulation(const std::vector<std::shared_ptr<Position>>& points);

    std::vector<std::shared_ptr<Vertex>> GetVertices();
    std::set<std::shared_ptr<Edge>> GetEdges();
    std::set<std::shared_ptr<Verge>> GetTriangles();

private:
    std::vector<std::shared_ptr<Vertex>> m_vertices;
    std::set<std::shared_ptr<Edge>> m_edges;
    std::set<std::shared_ptr<Verge>> m_triangles;

    std::shared_ptr<Triangulation> DivideAndConquer(const std::vector<std::shared_ptr<Vertex>>& vertices);
    std::shared_ptr<Triangulation> Merge(const std::shared_ptr<Triangulation>& left, const std::shared_ptr<Triangulation>& right);

    std::shared_ptr<Edge> CreateEdge(const std::shared_ptr<Vertex>& start, const std::shared_ptr<Vertex>& end);
    static std::shared_ptr<Verge> CreateTriangle(const std::shared_ptr<Edge>& e1, const std::shared_ptr<Edge>& e2, const std::shared_ptr<Edge>& e3);

    std::shared_ptr<Triangulation> TrivialTriangulation(const std::vector<std::shared_ptr<Vertex>>& vertices);
    std::shared_ptr<Triangulation> TripleTriangulation(const std::vector<std::shared_ptr<Vertex>>& vertices);
    std::shared_ptr<Triangulation> QuadrupleTriangulation(const std::vector<std::shared_ptr<Vertex>>& vertices);

    std::shared_ptr<Edge> FindUpTotalTangent(const std::vector<std::shared_ptr<Vertex>>& left, const std::vector<std::shared_ptr<Vertex>>& right);
    std::shared_ptr<Edge> FindDownTotalTangent(const std::vector<std::shared_ptr<Vertex>>& left, const std::vector<std::shared_ptr<Vertex>>& right);

    static std::shared_ptr<Vertex> FindMostUp(const std::vector<std::shared_ptr<Vertex>>& vertices);
    static std::shared_ptr<Vertex> FindMostDown(const std::vector<std::shared_ptr<Vertex>>& vertices);
    static std::shared_ptr<Vertex> FindMostLeft(const std::vector<std::shared_ptr<Vertex>>& vertices);
    static std::shared_ptr<Vertex> FindMostRight(const std::vector<std::shared_ptr<Vertex>>& vertices);

    static bool IsConvex(Position a, Position b, Position c, Position d);

    static double DefineDistance(Position a, Position b);
    static double DefineAngleCos(Position a, Position b, Position c);
    static double DefineAngleSin(Position a, Position b, Position c);

    std::shared_ptr<Edge> FindContiguousEdge(
        const std::shared_ptr<Vertex>& vertex,
        const std::vector<std::shared_ptr<Edge>>& edges);

    static std::shared_ptr<Edge> FindEdgeByPoints(const Position& point1, const Position& point2, const std::set<std::shared_ptr<Edge>>& edges);

    static std::shared_ptr<Vertex> FindClosestVertex(
        const std::shared_ptr<Edge>& edge,
        const std::vector<std::shared_ptr<Vertex>>& vertices,
        const std::set<std::shared_ptr<Vertex>>& exceptions);

    static bool IsDelaunayConditionSatisfied(const std::shared_ptr<Edge>& edge, const std::shared_ptr<Vertex>& vertex1, const std::shared_ptr<Vertex>& vertex2);

    static std::set<std::shared_ptr<Vertex>> FindExceptions(const std::vector<std::shared_ptr<Vertex>>& left, const std::vector<std::shared_ptr<Vertex>>& right);
    static bool AreEdgesIntersects(const std::shared_ptr<Edge>& v1, const std::shared_ptr<Edge>& v2);
    static bool IsVertexHigherThanEdge(const std::shared_ptr<Vertex>& vertex, const std::shared_ptr<Edge>& edge);
    std::vector<std::shared_ptr<Vertex>> SortTriangle(const std::vector<std::shared_ptr<Vertex>>& vertices);
    std::vector<std::shared_ptr<Vertex>> SortQuadrilateral(const std::vector<std::shared_ptr<Vertex>>& vertices);
    static double CrossProduct(const Position& a, const Position& b, const Position& c);

    static std::shared_ptr<Vertex> ClosestVertexOfEdge(const std::shared_ptr<Edge>& edge, const std::shared_ptr<Vertex>& vertex);
    std::shared_ptr<Vertex> ChooseVertexForNewEdge(const std::vector<std::shared_ptr<Vertex>>& leftVertices,
        const std::vector<std::shared_ptr<Vertex>>& rightVertices,
        const std::shared_ptr<Edge>& baseEdge,
        const std::shared_ptr<Vertex>& candidate);
    std::shared_ptr<Vertex> FindNextCandidate(
        const std::vector<std::shared_ptr<Vertex>>& vertices,
        const std::shared_ptr<Vertex>& currentEdgeStart,
        const std::shared_ptr<Vertex>& currentEdgeEnd,
        bool isRightPart);


    static bool IsPointInsideCircumcircle(const Position& a, const Position& b, const Position& c, const Position& p);
    static double DistanceToEdge(const Position& a, const Position& b, const Position& p);
};
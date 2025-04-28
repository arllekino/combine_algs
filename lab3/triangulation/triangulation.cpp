#include "./triangulation.h"

#include <iostream>
#include <ranges>
#include <set>

std::vector<std::shared_ptr<Vertex>> Triangulation::GetVertices()
{
    return m_vertices;
}

std::set<std::shared_ptr<Edge>> Triangulation::GetEdges()
{
    return m_edges;
}

std::set<std::shared_ptr<Verge>> Triangulation::GetTriangles()
{
    return m_triangles;
}

void Triangulation::DelaunayTriangulation(const std::vector<std::shared_ptr<Position>>& points)
{
    std::vector<std::shared_ptr<Vertex>> vertices;
    for (const auto& point : points)
    {
        auto vertex = std::make_shared<Vertex>();
        vertex->SetPosition({point->x, point->y});
        vertices.push_back(vertex);
    }

    std::ranges::sort(vertices,
    [](const std::shared_ptr<Vertex>& vertex1, const std::shared_ptr<Vertex>& vertex2) {
        return vertex1->GetPosition().x < vertex2->GetPosition().x || (vertex1->GetPosition().x == vertex2->GetPosition().x && vertex1->GetPosition().y < vertex2->GetPosition().y);
    });

    const auto result = DivideAndConquer(vertices);
    if (!result) return;

    this->m_vertices = result->m_vertices;
    this->m_triangles = result->m_triangles;
    this->m_edges = result->m_edges;
}

std::shared_ptr<Triangulation> Triangulation::DivideAndConquer(const std::vector<std::shared_ptr<Vertex>>& vertices)
{
    if (vertices.size() < 3) {
        return TrivialTriangulation(vertices);
    }
    if (vertices.size() == 3) {
        return TripleTriangulation(vertices);
    }
    if (vertices.size() == 4) {
        return QuadrupleTriangulation(vertices);
    }
    if (vertices.size() == 8) {
        const size_t mid = vertices.size() / 2;
        const std::vector<std::shared_ptr<Vertex>> left(vertices.begin(), vertices.begin() + mid);
        const std::vector<std::shared_ptr<Vertex>> right(vertices.begin() + mid, vertices.end());

        const auto leftTriangulation = Triangulation::DivideAndConquer(left);
        const auto rightTriangulation = Triangulation::DivideAndConquer(right);

        return Merge(leftTriangulation, rightTriangulation);
    }
    if (vertices.size() < 12) {
        const std::vector<std::shared_ptr<Vertex>> left(vertices.begin(), vertices.begin() + 3);
        const std::vector<std::shared_ptr<Vertex>> right(vertices.begin() + 3, vertices.end());

        const auto leftTriangulation = Triangulation::DivideAndConquer(left);
        const auto rightTriangulation = Triangulation::DivideAndConquer(right);

        return Merge(leftTriangulation, rightTriangulation);
    }
    // if vertices.size() >= 12
    const size_t mid = vertices.size() / 2;
    const std::vector<std::shared_ptr<Vertex>> left(vertices.begin(), vertices.begin() + mid);
    const std::vector<std::shared_ptr<Vertex>> right(vertices.begin() + mid, vertices.end());

    const auto leftTriangulation = Triangulation::DivideAndConquer(left);
    const auto rightTriangulation = Triangulation::DivideAndConquer(right);

    return Merge(leftTriangulation, rightTriangulation);
}

std::shared_ptr<Triangulation> Triangulation::Merge(
    const std::shared_ptr<Triangulation>& left,
    const std::shared_ptr<Triangulation>& right)
{
    auto result = std::make_shared<Triangulation>();

    result->m_vertices.insert(result->m_vertices.end(), left->m_vertices.begin(), left->m_vertices.end());
    result->m_vertices.insert(result->m_vertices.end(), right->m_vertices.begin(), right->m_vertices.end());
    result->m_edges.insert(left->m_edges.begin(), left->m_edges.end());
    result->m_edges.insert(right->m_edges.begin(), right->m_edges.end());
    result->m_triangles.insert(left->m_triangles.begin(), left->m_triangles.end());
    result->m_triangles.insert(right->m_triangles.begin(), right->m_triangles.end());

    auto upperTangent = FindUpTotalTangent(left->GetVertices(), right->GetVertices());
    auto lowerTangent = FindDownTotalTangent(left->GetVertices(), right->GetVertices());

    result->m_edges.insert(upperTangent);
    result->m_edges.insert(upperTangent->GetTwinEdge());
    result->m_edges.insert(lowerTangent);
    result->m_edges.insert(lowerTangent->GetTwinEdge());

    auto baseEdge = upperTangent;

    auto vertices = std::vector<std::shared_ptr<Vertex>>();
    vertices.insert(vertices.end(), left->m_vertices.begin(), left->m_vertices.end());
    vertices.insert(vertices.end(), right->m_vertices.begin(), right->m_vertices.end());

    std::set<std::shared_ptr<Vertex>> exceptionsVertex;
    exceptionsVertex.insert(upperTangent->GetEndPoint());
    exceptionsVertex.insert(upperTangent->GetTwinEdge()->GetEndPoint());

    while (!exceptionsVertex.contains(lowerTangent->GetEndPoint())
            || !exceptionsVertex.contains(lowerTangent->GetTwinEdge()->GetEndPoint()))
    {
        auto candidate = FindClosestVertex(baseEdge, vertices, exceptionsVertex);
        exceptionsVertex.insert(candidate);

        std::shared_ptr<Edge> newEdge;
        std::shared_ptr<Verge> newTriangle;

        if (std::ranges::find(left->m_vertices, candidate) != left->m_vertices.end()) {
            newEdge = CreateEdge(ChooseVertexForNewEdge(left->GetVertices(), right->GetVertices(), baseEdge, candidate), candidate);
            auto otherEdge = FindEdgeByPoints(baseEdge->GetTwinEdge()->GetEndPoint()->GetPosition(), newEdge->GetEndPoint()->GetPosition(), left->GetEdges());
            if (!otherEdge) {
                otherEdge = CreateEdge(baseEdge->GetTwinEdge()->GetEndPoint(), newEdge->GetEndPoint());
                result->m_edges.insert(otherEdge);
                result->m_edges.insert(otherEdge->GetTwinEdge());
            }

            newTriangle = CreateTriangle(baseEdge, newEdge, otherEdge);
        } else {
            newEdge = CreateEdge(candidate, ChooseVertexForNewEdge(left->GetVertices(), right->GetVertices(), baseEdge, candidate));
            auto otherEdge = FindEdgeByPoints(baseEdge->GetEndPoint()->GetPosition(), newEdge->GetTwinEdge()->GetEndPoint()->GetPosition(), right->GetEdges());
            if (!otherEdge) {
                otherEdge = CreateEdge(baseEdge->GetEndPoint(), newEdge->GetTwinEdge()->GetEndPoint());
                result->m_edges.insert(otherEdge);
                result->m_edges.insert(otherEdge->GetTwinEdge());
            }
            newTriangle = CreateTriangle(baseEdge, newEdge, otherEdge);
        }

        std::set<std::shared_ptr<Edge>> edgesToDelete;
        for (const auto& edge : result->m_edges) {
            if (AreEdgesIntersects(newEdge, edge) && !edgesToDelete.contains(edge)) {
                edgesToDelete.insert(edge);
            }
        }

        for (const auto& e : edgesToDelete) {
            result->m_edges.erase(e);
        }

        result->m_edges.insert({newEdge, newEdge->GetTwinEdge()});
        result->m_triangles.insert(newTriangle);

        baseEdge = newEdge->GetTwinEdge();
    }

    return result;
}

std::shared_ptr<Edge> Triangulation::CreateEdge(const std::shared_ptr<Vertex>& start, const std::shared_ptr<Vertex>& end)
{
    EdgeState state;
    state.endPoint = end;
    auto edge = std::make_shared<Edge>(state);

    EdgeState twinState;
    twinState.endPoint = start;
    const auto twinEdge = std::make_shared<Edge>(twinState);

    edge->SetTwinEdge(twinEdge);
    twinEdge->SetTwinEdge(edge);

    start->AddAdjacentVertex(end);
    end->AddAdjacentVertex(start);

    return edge;
}

std::shared_ptr<Verge> Triangulation::CreateTriangle(
    const std::shared_ptr<Edge>& e1,
    const std::shared_ptr<Edge>& e2,
    const std::shared_ptr<Edge>& e3)
{

    auto v1 = e1->GetEndPoint();
    auto v2 = e2->GetEndPoint();
    auto v3 = e3->GetEndPoint();

    auto triangle = std::make_shared<Verge>(v1, v2, v3, nullptr, nullptr, nullptr);

    e1->SetNextEdge(e2);
    e2->SetNextEdge(e3);
    e3->SetNextEdge(e1);

    e1->SetTriangle(triangle);
    e2->SetTriangle(triangle);
    e3->SetTriangle(triangle);

    return triangle;
}

std::shared_ptr<Triangulation> Triangulation::TrivialTriangulation(const std::vector<std::shared_ptr<Vertex>>& vertices)
{
    auto result = std::make_shared<Triangulation>();
    result->m_vertices = vertices;
    if (vertices.size() == 2) {
        auto edge = CreateEdge(vertices[0], vertices[1]);
        result->m_edges.insert(edge);
        result->m_edges.insert(edge->GetTwinEdge());
    }
    return result;
}

std::shared_ptr<Triangulation> Triangulation::TripleTriangulation(const std::vector<std::shared_ptr<Vertex>>& vertices)
{
    auto result = std::make_shared<Triangulation>();
    result->m_vertices = vertices;

    auto sortedVertices = SortTriangle(vertices);

    auto edge1 = CreateEdge(sortedVertices[0], sortedVertices[1]);
    auto edge2 = CreateEdge(sortedVertices[1], sortedVertices[2]);
    auto edge3 = CreateEdge(sortedVertices[2], sortedVertices[0]);

    edge1->SetNextEdge(edge2);
    edge2->SetNextEdge(edge3);
    edge3->SetNextEdge(edge1);

    auto triangle = CreateTriangle(edge1, edge2, edge3);
    if (triangle)
    {
        edge1->SetTriangle(triangle);
        edge2->SetTriangle(triangle);
        edge3->SetTriangle(triangle);
        result->m_triangles.insert(triangle);

        if (edge1->GetTwinEdge() && edge2->GetTwinEdge() && edge3->GetTwinEdge())
        {
            result->m_edges.insert({
                edge1, edge1->GetTwinEdge(),
                edge2, edge2->GetTwinEdge(),
                edge3, edge3->GetTwinEdge()
            });
        }
    }
    return result;
}

std::shared_ptr<Triangulation> Triangulation::QuadrupleTriangulation(const std::vector<std::shared_ptr<Vertex>>& vertices)
{
    auto result = std::make_shared<Triangulation>();

    result->m_vertices = vertices;
    auto quadrilateralSorted = SortQuadrilateral(vertices);

    if (IsConvex(quadrilateralSorted[0]->GetPosition(), quadrilateralSorted[1]->GetPosition(), quadrilateralSorted[2]->GetPosition(), quadrilateralSorted[3]->GetPosition())) {
        const auto edge1 = CreateEdge(quadrilateralSorted[0], quadrilateralSorted[1]);
        const auto edge2 = CreateEdge(quadrilateralSorted[1], quadrilateralSorted[2]);
        const auto edge3 = CreateEdge(quadrilateralSorted[2], quadrilateralSorted[3]);
        const auto edge4 = CreateEdge(quadrilateralSorted[3], quadrilateralSorted[0]);

        const auto angle123 = DefineAngleCos(quadrilateralSorted[0]->GetPosition(), quadrilateralSorted[1]->GetPosition(), quadrilateralSorted[2]->GetPosition());
        const auto angle234 = DefineAngleCos(quadrilateralSorted[1]->GetPosition(), quadrilateralSorted[2]->GetPosition(), quadrilateralSorted[3]->GetPosition());
        const auto angle341 = DefineAngleCos(quadrilateralSorted[2]->GetPosition(), quadrilateralSorted[3]->GetPosition(), quadrilateralSorted[0]->GetPosition());
        const auto angle412 = DefineAngleCos(quadrilateralSorted[3]->GetPosition(), quadrilateralSorted[0]->GetPosition(), quadrilateralSorted[1]->GetPosition());

        std::shared_ptr<Edge> commonEdge;
        if (angle412 + angle234 < angle123 + angle341) {
            commonEdge = CreateEdge(quadrilateralSorted[0], quadrilateralSorted[2]);

            const auto triangle1 = CreateTriangle(edge1, commonEdge, edge4);
            const auto triangle2 = CreateTriangle(edge2, commonEdge, edge3);

            edge1->SetTriangle(triangle1);
            edge2->SetTriangle(triangle2);
            edge3->SetTriangle(triangle2);
            edge4->SetTriangle(triangle1);
            commonEdge->SetTriangle(triangle1);
            commonEdge->GetTwinEdge()->SetTriangle(triangle2);

            result->m_triangles.insert(triangle1);
            result->m_triangles.insert(triangle2);
        } else {
            commonEdge = CreateEdge(quadrilateralSorted[1], quadrilateralSorted[3]);
            const auto triangle1 = CreateTriangle(edge1, commonEdge, edge2);
            const auto triangle2 = CreateTriangle(edge3, commonEdge, edge4);

            edge1->SetTriangle(triangle1);
            edge2->SetTriangle(triangle1);
            edge3->SetTriangle(triangle2);
            edge4->SetTriangle(triangle2);
            commonEdge->SetTriangle(triangle2);
            commonEdge->GetTwinEdge()->SetTriangle(triangle1);

            result->m_triangles.insert(triangle1);
            result->m_triangles.insert(triangle2);
        }

        result->m_edges.insert({
            edge1, edge1->GetTwinEdge(),
            edge2, edge2->GetTwinEdge(),
            edge3, edge3->GetTwinEdge(),
            edge4, edge4->GetTwinEdge(),
            commonEdge, commonEdge->GetTwinEdge(),
        });

    } else {
        const auto edge1 = CreateEdge(vertices[0], vertices[1]);
        const auto edge2 = CreateEdge(vertices[0], vertices[2]);
        const auto edge3 = CreateEdge(vertices[0], vertices[3]);
        const auto edge4 = CreateEdge(vertices[1], vertices[2]);
        const auto edge5 = CreateEdge(vertices[1], vertices[3]);
        const auto edge6 = CreateEdge(vertices[2], vertices[3]);

        const auto triangle1 = CreateTriangle(edge1, edge3, edge5);
        const auto triangle2 = CreateTriangle(edge4, edge5, edge6);
        const auto triangle3 = CreateTriangle(edge2, edge3, edge6);

        edge1->SetTriangle(triangle1);
        edge2->GetTwinEdge()->SetTriangle(triangle3);
        edge3->SetTriangle(triangle3);
        edge3->GetTwinEdge()->SetTriangle(triangle1);
        edge4->SetTriangle(triangle2);
        edge5->SetTriangle(triangle1);
        edge5->GetTwinEdge()->SetTriangle(triangle2);
        edge6->SetTriangle(triangle2);
        edge6->GetTwinEdge()->SetTriangle(triangle3);
        result->m_edges.insert({
            edge1, edge1->GetTwinEdge(),
            edge2, edge2->GetTwinEdge(),
            edge3, edge3->GetTwinEdge(),
            edge4, edge4->GetTwinEdge(),
            edge5, edge5->GetTwinEdge(),
            edge6, edge6->GetTwinEdge()
        });
        result->m_triangles.insert({
            triangle1, triangle2, triangle3
        });
    }

    return result;
}

double cross(const Position& o, const Position& a, const Position& b) {
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

std::shared_ptr<Edge> Triangulation::FindUpTotalTangent(
    const std::vector<std::shared_ptr<Vertex>>& left,
    const std::vector<std::shared_ptr<Vertex>>& right)
{
    auto leftMostRight = FindMostRight(left);
    auto rightMostLeft = FindMostLeft(right);

    while (true) {
        bool changed = false;
        auto nextRight = FindNextCandidate(right, leftMostRight, rightMostLeft, true);
        if (nextRight) {
            rightMostLeft = nextRight;
            changed = true;
        }
        auto nextLeft = FindNextCandidate(left, rightMostLeft, leftMostRight, false);
        if (nextLeft) {
            leftMostRight = nextLeft;
            changed = true;
        }
        if (!changed) break;
    }

    return CreateEdge(leftMostRight, rightMostLeft);
}

std::shared_ptr<Edge> Triangulation::FindDownTotalTangent(
    const std::vector<std::shared_ptr<Vertex>>& left,
    const std::vector<std::shared_ptr<Vertex>>& right)
{
    auto leftMostRight = FindMostRight(left);
    auto rightMostLeft = FindMostLeft(right);

    while (true) {
        bool changed = false;

        auto nextRight = FindNextCandidate(
            right, leftMostRight, rightMostLeft, false
        );
        if (nextRight) {
            rightMostLeft = nextRight;
            changed = true;
        }

        auto nextLeft = FindNextCandidate(
            left, rightMostLeft, leftMostRight, true
        );
        if (nextLeft) {
            leftMostRight = nextLeft;
            changed = true;
        }

        if (!changed) break;
    }

    return CreateEdge(leftMostRight, rightMostLeft);
}

std::shared_ptr<Vertex> Triangulation::FindMostUp(const std::vector<std::shared_ptr<Vertex>>& vertices)
{
    return *std::ranges::min_element(vertices.begin(), vertices.end(),
       [](const auto& v1, const auto& v2) {
           return v1->GetPosition().y < v2->GetPosition().y;
       });
}

std::shared_ptr<Vertex> Triangulation::FindMostDown(const std::vector<std::shared_ptr<Vertex>>& vertices)
{
    return *std::ranges::max_element(vertices.begin(), vertices.end(),
       [](const auto& v1, const auto& v2) {
           return v1->GetPosition().y < v2->GetPosition().y;
       });
}

std::shared_ptr<Vertex> Triangulation::FindMostLeft(const std::vector<std::shared_ptr<Vertex>>& vertices)
{
    return *std::ranges::min_element(vertices.begin(), vertices.end(),
       [](const auto& v1, const auto& v2) {
           return v1->GetPosition().x < v2->GetPosition().x;
       });
}

std::shared_ptr<Vertex> Triangulation::FindMostRight(const std::vector<std::shared_ptr<Vertex>>& vertices)
{
    return *std::ranges::max_element(vertices.begin(), vertices.end(),
       [](const auto& v1, const auto& v2) {
           return v1->GetPosition().x < v2->GetPosition().x;
       });
}

bool Triangulation::IsConvex(Position a, Position b, Position c, Position d)
{
    double cross1 = CrossProduct(a, b, c);
    double cross2 = CrossProduct(b, c, d);
    double cross3 = CrossProduct(c, d, a);
    double cross4 = CrossProduct(d, a, b);

    return (cross1 * cross2 > 0) && (cross2 * cross3 > 0) && (cross3 * cross4 > 0);
}

double Triangulation::DefineDistance(Position a, Position b)
{
    return sqrt(std::pow(b.x - a.x, 2) + std::pow(b.y - a.y, 2));
}

double Triangulation::DefineAngleCos(Position a, Position b, Position c)
{
    double baX = a.x - b.x;
    double baY = a.y - b.y;
    double bcX = c.x - b.x;
    double bcY = c.y - b.y;

    double dotProduct = baX * bcX + baY * bcY;

    double ba_length = sqrt(baX * baX + baY * baY);
    double bc_length = sqrt(bcX * bcX + bcY * bcY);

    if (ba_length == 0.0 || bc_length == 0.0) {
        return 1.0;
    }

    return dotProduct / (ba_length * bc_length);
}

double Triangulation::DefineAngleSin(Position a, Position b, Position c)
{
    double baX = a.x - b.x;
    double baY = a.y - b.y;
    double bcX = c.x - b.x;
    double bcY = c.y - b.y;

    double crossProduct = baX * bcY - baY * bcX;

    double ba_length = sqrt(baX * baX + baY * baY);
    double bc_length = sqrt(bcX * bcX + bcY * bcY);

    if (ba_length == 0.0 || bc_length == 0.0) {
        return 0.0;
    }

    return crossProduct / (ba_length * bc_length);
}

bool InCircle(const Position& a, const Position& b, const Position& c, const Position& d)
{
    double ax = a.x - d.x;
    double ay = a.y - d.y;
    double bx = b.x - d.x;
    double by = b.y - d.y;
    double cx = c.x - d.x;
    double cy = c.y - d.y;

    double det = (ax * ax + ay * ay) * (bx * cy - cx * by)
               - (bx * bx + by * by) * (ax * cy - cx * ay)
               + (cx * cx + cy * cy) * (ax * by - bx * ay);

    return det < 0;
}

std::shared_ptr<Vertex> Triangulation::FindClosestVertex(
    const std::shared_ptr<Edge>& edge,
    const std::vector<std::shared_ptr<Vertex>>& vertices,
    const std::set<std::shared_ptr<Vertex>>& exceptions)
{
    auto a = edge->GetTwinEdge()->GetEndPoint()->GetPosition();
    auto c = edge->GetEndPoint()->GetPosition();

    std::shared_ptr<Vertex> bestCandidate = nullptr;

    for (const auto& vertex : vertices)
    {
        if (exceptions.contains(vertex))
            continue;

        if (!bestCandidate)
        {
            bestCandidate = vertex;
            continue;
        }

        if (InCircle(a, c, bestCandidate->GetPosition(), vertex->GetPosition()))
        {
            bestCandidate = vertex;
        }
    }

    return bestCandidate;
}


std::set<std::shared_ptr<Vertex>> Triangulation::FindExceptions(const std::vector<std::shared_ptr<Vertex>>& left, const std::vector<std::shared_ptr<Vertex>>& right)
{
    std::set<std::shared_ptr<Vertex>> exceptions;

    auto leftUp = FindMostUp(left);
    auto leftDown = FindMostDown(left);
    auto leftEdgePoint = leftUp->GetPosition().x < leftDown->GetPosition().x ? leftUp->GetPosition() : leftDown->GetPosition();

    auto rightUp = FindMostUp(right);
    auto rightDown = FindMostDown(right);
    auto rightEdgePoint = rightUp->GetPosition().x < rightDown->GetPosition().x ? rightUp->GetPosition() : rightDown->GetPosition();

    for (const auto& vertex : left) {
        if (vertex->GetPosition().x < leftEdgePoint.x)
            exceptions.insert(vertex);
    }

    for (const auto& vertex : right) {
        if (vertex->GetPosition().x > rightEdgePoint.x)
            exceptions.insert(vertex);
    }

    return exceptions;
}

bool Triangulation::AreEdgesIntersects(const std::shared_ptr<Edge>& edge1, const std::shared_ptr<Edge>& edge2) {
    auto cross = [](Position p, Position q, Position r) {
        return (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x);
    };

    Position a = edge1->GetTwinEdge()->GetEndPoint()->GetPosition();
    Position b = edge1->GetEndPoint()->GetPosition();
    Position c = edge2->GetTwinEdge()->GetEndPoint()->GetPosition();
    Position d = edge2->GetEndPoint()->GetPosition();

    if (a == c || a == d || b == c || b == d) {
        return false;
    }

    double d1 = cross(a, b, c);
    double d2 = cross(a, b, d);
    double d3 = cross(c, d, a);
    double d4 = cross(c, d, b);

    if ((d1 * d2 < 0) && (d3 * d4 < 0)) {
        return true;
    }

    if (d1 == 0 && d2 == 0 && d3 == 0 && d4 == 0) {
        bool xOverlap = std::max(a.x, b.x) >= std::min(c.x, d.x) &&
                       std::max(c.x, d.x) >= std::min(a.x, b.x);
        bool yOverlap = std::max(a.y, b.y) >= std::min(c.y, d.y) &&
                       std::max(c.y, d.y) >= std::min(a.y, b.y);
        return xOverlap && yOverlap;
    }

    return false;
}

std::shared_ptr<Edge> Triangulation::FindContiguousEdge(const std::shared_ptr<Vertex>& vertex, const std::vector<std::shared_ptr<Edge>>& edges)
{
    std::shared_ptr<Edge> contiguousEdge;
    for (const auto& edge : edges) {
        if (edge->GetEndPoint() == vertex || edge->GetTwinEdge()->GetEndPoint() == vertex) {
            contiguousEdge = edge;
            break;
        }
    }

    return contiguousEdge->GetNextEdge()->GetNextEdge();
}

std::shared_ptr<Edge> Triangulation::FindEdgeByPoints(const Position& point1, const Position& point2, const std::set<std::shared_ptr<Edge>>& edges)
{
    for (const auto& edge : edges) {
        if (edge->GetTwinEdge()->GetEndPoint()->GetPosition() == point1
            && edge->GetEndPoint()->GetPosition() == point2
            || edge->GetEndPoint()->GetPosition() == point1
            && edge->GetTwinEdge()->GetEndPoint()->GetPosition() == point2) {
            return edge;
        }
    }
    return nullptr;
}

bool Triangulation::IsVertexHigherThanEdge(const std::shared_ptr<Vertex>& vertex, const std::shared_ptr<Edge>& edge)
{
    auto a = edge->GetTwinEdge()->GetEndPoint()->GetPosition();
    auto b = edge->GetEndPoint()->GetPosition();
    auto p = vertex->GetPosition();

    auto ab = b - a;
    auto ap = p - a;

    auto cross = ab.x * ap.y - ab.y * ap.x;

    return cross > 0;
}

double Triangulation::CrossProduct(const Position& a, const Position& b, const Position& c)
{
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

std::vector<std::shared_ptr<Vertex>> Triangulation::SortTriangle(const std::vector<std::shared_ptr<Vertex>>& vertices)
{
    auto sorted = vertices;

    std::ranges::sort(sorted.begin(), sorted.end(), [](const auto& a, const auto& b) {
        if (a->GetPosition().x != b->GetPosition().x)
            return a->GetPosition().x < b->GetPosition().x;
        return a->GetPosition().y < b->GetPosition().y;
    });

    Position p0 = sorted[0]->GetPosition();
    std::ranges::sort(sorted.begin() + 1, sorted.end(), [p0](const auto& a, const auto& b) {
        Position pa = a->GetPosition();
        Position pb = b->GetPosition();
        double cross = (pa.x - p0.x) * (pb.y - p0.y) - (pa.y - p0.y) * (pb.x - p0.x);
        return cross < 0;
    });

    return sorted;
}

std::vector<std::shared_ptr<Vertex>> Triangulation::SortQuadrilateral(const std::vector<std::shared_ptr<Vertex>>& vertices)
{
    std::vector<std::shared_ptr<Vertex>> result(4);

    auto v = vertices;

    std::ranges::sort(v.begin(), v.end(), [](const auto& a, const auto& b) {
        if (a->GetPosition().x != b->GetPosition().x)
            return a->GetPosition().x < b->GetPosition().x;
        return a->GetPosition().y < b->GetPosition().y;
    });

    if (v[0]->GetPosition().y < v[1]->GetPosition().y) {
        result[0] = v[0];
        result[1] = v[1];
    } else {
        result[0] = v[1];
        result[1] = v[0];
    }

    if (v[2]->GetPosition().y > v[3]->GetPosition().y) {
        result[2] = v[2];
        result[3] = v[3];
    } else {
        result[2] = v[3];
        result[3] = v[2];
    }

    return result;
}

bool Triangulation::IsDelaunayConditionSatisfied(const std::shared_ptr<Edge>& edge, const std::shared_ptr<Vertex>& vertex1, const std::shared_ptr<Vertex>& vertex2)
{
    auto p0 = edge->GetEndPoint()->GetPosition();
    auto p1 = edge->GetTwinEdge()->GetEndPoint()->GetPosition();
    auto p2 = vertex2->GetPosition();
    auto p3 = vertex1->GetPosition();

    auto Sa = (p0.x - p1.x) * (p0.x - p3.x) + (p0.y - p1.y) * (p0.y - p3.y);
    auto Sb= (p2.x - p1.x) * (p2.x - p3.x) + (p2.y - p1.y) * (p2.y - p3.y);

    return Sa >= 0 && Sb >= 0;
}

std::shared_ptr<Vertex> Triangulation::ClosestVertexOfEdge(const std::shared_ptr<Edge>& edge, const std::shared_ptr<Vertex>& vertex)
{
    auto a = DefineDistance(edge->GetEndPoint()->GetPosition(), vertex->GetPosition());
    auto b = DefineDistance(edge->GetTwinEdge()->GetEndPoint()->GetPosition(), vertex->GetPosition());

    return a < b ? edge->GetEndPoint() : edge->GetTwinEdge()->GetEndPoint();
}

std::shared_ptr<Vertex> Triangulation::ChooseVertexForNewEdge(const std::vector<std::shared_ptr<Vertex>>& leftVertices,
    const std::vector<std::shared_ptr<Vertex>>& rightVertices,
    const std::shared_ptr<Edge>& baseEdge,
    const std::shared_ptr<Vertex>& candidate)
{
    if (std::find(leftVertices.begin(), leftVertices.end(), baseEdge->GetEndPoint()) != leftVertices.end()
        && std::find(leftVertices.begin(), leftVertices.end(), baseEdge->GetTwinEdge()->GetEndPoint()) != leftVertices.end()
        || std::find(rightVertices.begin(), rightVertices.end(), baseEdge->GetEndPoint()) != rightVertices.end()
        && std::find(rightVertices.begin(), rightVertices.end(), baseEdge->GetTwinEdge()->GetEndPoint()) != rightVertices.end()) {
        return ClosestVertexOfEdge(baseEdge, candidate);
    }

    if (std::find(leftVertices.begin(), leftVertices.end(), candidate) != leftVertices.end()) {
        if (std::find(rightVertices.begin(), rightVertices.end(), baseEdge->GetEndPoint()) != rightVertices.end()) {
            return baseEdge->GetEndPoint();
        }
        return baseEdge->GetTwinEdge()->GetEndPoint();
    }
    if (std::find(leftVertices.begin(), leftVertices.end(), baseEdge->GetEndPoint()) != leftVertices.end()) {
        return baseEdge->GetEndPoint();
    }
    return baseEdge->GetTwinEdge()->GetEndPoint();
}

std::shared_ptr<Vertex> Triangulation::FindNextCandidate(
    const std::vector<std::shared_ptr<Vertex>>& vertices,
    const std::shared_ptr<Vertex>& currentEdgeStart,
    const std::shared_ptr<Vertex>& currentEdgeEnd,
    bool isRightPart)
{
    std::shared_ptr<Vertex> nextCandidate = nullptr;
    const Position& a = currentEdgeStart->GetPosition();
    const Position& b = currentEdgeEnd->GetPosition();

    for (const auto& vertex : vertices) {
        if (vertex == currentEdgeStart || vertex == currentEdgeEnd) continue;

        const Position& p = vertex->GetPosition();
        double cross = (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);

        if ((isRightPart && cross > 0) || (!isRightPart && cross < 0)) {
            if (!nextCandidate) {
                nextCandidate = vertex;
            } else {
                double crossWithPrev = (nextCandidate->GetPosition().x - a.x) * (p.y - a.y)
                                     - (nextCandidate->GetPosition().y - a.y) * (p.x - a.x);
                if ((isRightPart && crossWithPrev < 0) || (!isRightPart && crossWithPrev > 0)) {
                    nextCandidate = vertex;
                }
            }
        }
    }

    return nextCandidate;
}
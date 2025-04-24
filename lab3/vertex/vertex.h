#pragma once

#include <vector>

class Vertex;

struct Position {
    double x = 0;
    double y = 0;

    bool operator==(const Position& other) const {
        return x == other.x && y == other.y;
    }
    Position operator+(const Position& other) const {
        return Position{x + other.x, y + other.y};
    }
    Position operator-(const Position& other) const {
        return Position{x - other.x, y - other.y};
    }
    Position operator/(const double other) const {
        return Position{x / other, y / other};
    }
};

using VertexList = std::vector<std::shared_ptr<Vertex>>;

struct VertexState
{
    Position position;
    int count = 0;
    VertexList adjacentVertexList;
};

class Vertex
{
public:
    Position GetPosition() const;
    VertexList GetAdjacentVertexList();

    void SetPosition(Position position);
    void AddAdjacentVertex(const std::shared_ptr<Vertex> &vertex);
    void DeleteFromAdjacentVertex(const std::shared_ptr<Vertex>& vertex);

    bool operator==(const Vertex& other) const;
private:
    VertexState m_state;
};
#include "triangulation/triangulation.h"

#include <iostream>
#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <random>

std::vector<sf::VertexArray> ConvertEdgesToLines(const std::set<std::shared_ptr<Edge>>& edges)
{
    auto result = std::vector<sf::VertexArray>();
    const float scale = 10.0f;
    const float offsetX = 400.0f;
    const float offsetY = 300.0f;

    for (const auto& edge : edges) {
        auto from = edge->GetTwinEdge()->GetEndPoint()->GetPosition();
        auto to = edge->GetEndPoint()->GetPosition();

        sf::VertexArray line(sf::PrimitiveType::Lines, 2);

        line[0].position = sf::Vector2f(from.x * scale + offsetX, -from.y * scale + offsetY);
        line[1].position = sf::Vector2f(to.x * scale + offsetX, -to.y * scale + offsetY);

        line[0].color = sf::Color::Cyan;
        line[1].color = sf::Color::Cyan;

        result.push_back(line);
    }
    return result;
}

std::vector<sf::CircleShape> ConvertToPoints(const std::vector<std::shared_ptr<Vertex>>& vertices)
{
    std::vector<sf::CircleShape> result;
    const float scale = 10.0f;
    const float offsetX = 400.0f;
    const float offsetY = 300.0f;

    for (const auto& vertex : vertices)
    {
        auto point = vertex->GetPosition();

        const auto radius = 3.0f;
        sf::CircleShape circle(radius);
        circle.setFillColor(sf::Color::Red);
        circle.setPosition({static_cast<float>(point.x * scale + offsetX - radius/2), static_cast<float>(-point.y * scale + offsetY - radius/2)});

        result.push_back(circle);
    }

    return result;
}



int main() {
    Triangulation triangulation;

    std::vector<std::shared_ptr<Position>> points
    = {
        std::make_shared<Position>(-35, 25),
        std::make_shared<Position>(-20, 15),
        std::make_shared<Position>(-30, 15),
        std::make_shared<Position>(-35, 10),
        std::make_shared<Position>(0, 0),
        std::make_shared<Position>(1, 10),
        std::make_shared<Position>(2, 1),
        std::make_shared<Position>(6, 6),
        std::make_shared<Position>(10, 0),
        std::make_shared<Position>(10, 6),
        std::make_shared<Position>(10, 10),
        std::make_shared<Position>(15, 15),
        std::make_shared<Position>(15, 5),
        std::make_shared<Position>(16, 0),
        std::make_shared<Position>(16, 10),
        std::make_shared<Position>(22, 2),
        std::make_shared<Position>(24, 10),
    };

    // const float scale = 10.0f;
    // const float offsetX = 400.0f;
    // const float offsetY = 300.0f;
    // const int windowWidth = 800;
    // const int windowHeight = 600;
    //
    // std::mt19937 rng(std::time(nullptr));
    // std::uniform_real_distribution<float> distX(-40.0f, 40.0f);
    // std::uniform_real_distribution<float> distY(-30.0f, 30.0f);
    //
    // for(int i = 0; i < 100; ++i) {
    //     float x = distX(rng);
    //     float y = distY(rng);
    //
    //     float screenX = x * scale + offsetX;
    //     float screenY = -y * scale + offsetY;
    //
    //     if(screenX >= 0 && screenX <= windowWidth &&
    //        screenY >= 0 && screenY <= windowHeight) {
    //         points.emplace_back(std::make_shared<Position>(x, y));
    //        }
    // }

    triangulation.DelaunayTriangulation(points);
    //
    // std::cout << "Triangles count: " << triangulation.GetTriangles().size() << std::endl;
    // std::cout << "Edges count: " << triangulation.GetEdges().size() << std::endl;
    //
    const auto edges = triangulation.GetEdges();
    if (edges.empty()) {
        std::cerr << "Error: No edges to draw!" << std::endl;
        return -1;
    }

    auto drawableEdges = ConvertEdgesToLines(edges);
    auto drawablePoints = ConvertToPoints(triangulation.GetVertices());

    sf::ContextSettings settings;
    settings.antiAliasingLevel = 4;

    sf::RenderWindow window(
        sf::VideoMode({800, 600}),
        "Triangulation",
        sf::State::Windowed,
        settings
    );

    while (window.isOpen()) {
        while (const auto event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>())
            {
                window.close();
            }
        }

        window.clear(sf::Color::Black);

        for (const auto& edge : drawableEdges) {
            window.draw(edge);
        }
        for (const auto& point : drawablePoints) {
            window.draw(point);
        }

        window.display();
    }

    return 0;
}
#include <iostream>
#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <opencv2/opencv.hpp>
#include <random>
#include "triangulation/triangulation.h"

sf::Vector2f ConvertToScreenCoords(const Position& point,
                                 const sf::Vector2f& center,
                                 float scale)
{
    return sf::Vector2f(
        point.x * scale + center.x - 400,
        -point.y * scale + center.y + 300
    );
}

std::vector<sf::VertexArray> ConvertEdgesToLines(const std::set<std::shared_ptr<Edge>>& edges,
                                               const sf::Vector2f& center,
                                               float scale)
{
    std::vector<sf::VertexArray> result;
    const sf::Color lineColor = sf::Color::Cyan;

    for (const auto& edge : edges) {
        auto from = edge->GetTwinEdge()->GetEndPoint()->GetPosition();
        auto to = edge->GetEndPoint()->GetPosition();

        sf::VertexArray line(sf::PrimitiveType::Lines, 2);
        line[0].position = ConvertToScreenCoords(from, center, scale);
        line[1].position = ConvertToScreenCoords(to, center, scale);
        line[0].color = lineColor;
        line[1].color = lineColor;

        result.push_back(line);
    }
    return result;
}

std::vector<sf::CircleShape> ConvertToPoints(const std::vector<std::shared_ptr<Vertex>>& vertices,
                                           const sf::Vector2f& center,
                                           float scale)
{
    std::vector<sf::CircleShape> result;
    const float pointRadius = 3.0f;
    const sf::Color pointColor = sf::Color::Red;

    for (const auto& vertex : vertices) {
        auto point = vertex->GetPosition();
        sf::CircleShape circle(pointRadius);
        circle.setFillColor(pointColor);

        sf::Vector2f screenPos = ConvertToScreenCoords(point, center, scale);
        circle.setPosition(screenPos - sf::Vector2f(pointRadius, pointRadius));

        result.push_back(circle);
    }
    return result;
}

int main() {
    cv::Mat img = cv::imread("/Users/miroslav.aktuganov/Desktop/home_projects/combine_algs/lab3/images/2025-04-29 13.11.57.jpg", cv::IMREAD_GRAYSCALE);
    if (img.empty()) {
        std::cerr << "Failed to load image!" << std::endl;
        return -1;
    }

    auto detector = cv::FastFeatureDetector::create();
    std::vector<cv::KeyPoint> keypoints;
    detector->detect(img, keypoints);
    std::cout << "Detected " << keypoints.size() << " keypoints" << std::endl;

    std::vector<std::shared_ptr<Position>> points;
    for (const auto& kp : keypoints) {
        points.push_back(std::make_shared<Position>(kp.pt.x, kp.pt.y));
    }

    const unsigned int windowWidth = 800;
    const unsigned int windowHeight = 600;
    sf::RenderWindow window(
        sf::VideoMode({windowWidth, windowHeight}),
        "Triangulation Visualization",
        sf::State::Windowed,
        sf::ContextSettings(0, 0, 4)
    );

    float scale = 0.5f;
    sf::Vector2f center(windowWidth / 2.0f, windowHeight / 2.0f);

    // Триангуляция
    Triangulation triangulation;
    triangulation.DelaunayTriangulation(points);
    const auto edges = triangulation.GetEdges();
    const auto vertices = triangulation.GetVertices();

    // Преобразование для отрисовки
    auto drawableEdges = ConvertEdgesToLines(edges, center, scale);
    auto drawablePoints = ConvertToPoints(vertices, center, scale);

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
        // for (const auto& point : drawablePoints) {
        //     window.draw(point);
        // }

        window.display();
    }

    return 0;
}
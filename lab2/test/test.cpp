#include <iostream>
#include <catch2/catch_test_macros.hpp>

#include "catch2/internal/catch_compiler_capabilities.hpp"
#include "catch2/internal/catch_decomposer.hpp"
#include "catch2/internal/catch_test_macro_impl.hpp"
#include "catch2/internal/catch_test_registry.hpp"

#include "../graph.h"

TEST_CASE("Lection test")
{
    auto edge1 = GraphEdge(0, 1, 1);
    auto edge2 = GraphEdge(1, 10, 1);
    auto edge3 = GraphEdge(1, 2, 1);
    auto edge4 = GraphEdge(2, 4, 1);
    auto edge5 = GraphEdge(2, 3, 1);
    auto edge6 = GraphEdge(4, 3, 1);
    auto edge7 = GraphEdge(3, 5, 1);
    auto edge8 = GraphEdge(0, 6, 1);
    auto edge9 = GraphEdge(6, 7, 1);
    auto edge10 = GraphEdge(6, 8, 1);
    auto edge11 = GraphEdge(7, 8, 1);
    auto edge12 = GraphEdge(8, 9, 1);

    auto graph = Graph(11, std::vector<GraphEdge>{edge1, edge2, edge3, edge4, edge5, edge6, edge7, edge8, edge9, edge10, edge11, edge12});
    if (!graph.IsError()) {
        auto points = graph.FindArticulationPoint();
        graph.PrintPoints(points);
    }
    std::cout << "-------" << std::endl;

}

TEST_CASE("Full graph test")
{
    auto edge1 = GraphEdge(0, 1, 1);
    auto edge2 = GraphEdge(0, 3, 1);
    auto edge3 = GraphEdge(1, 2, 1);
    auto edge4 = GraphEdge(1, 4, 1);
    auto edge5 = GraphEdge(2, 5, 1);
    auto edge6 = GraphEdge(2, 6, 1);
    auto edge7 = GraphEdge(3, 4, 1);
    auto edge8 = GraphEdge(3, 7, 1);
    auto edge9 = GraphEdge(4, 5, 1);
    auto edge10 = GraphEdge(4, 8, 1);
    auto edge11 = GraphEdge(5, 9, 1);
    auto edge12 = GraphEdge(6, 7, 1);
    auto edge13 = GraphEdge(7, 8, 1);
    auto edge14 = GraphEdge(8, 9, 1);

    auto graph = Graph(10, std::vector<GraphEdge>{edge1, edge2, edge3, edge4, edge5, edge6, edge7, edge8, edge9, edge10, edge11, edge12, edge13, edge14});
    if (!graph.IsError()) {
        auto points = graph.FindArticulationPoint();
        graph.PrintPoints(points);
    }
    std::cout << "-------" << std::endl;
}

TEST_CASE("Tree test")
{
    auto edge1 = GraphEdge(0, 1, 1);
    auto edge2 = GraphEdge(1, 3, 1);
    auto edge3 = GraphEdge(1, 4, 1);
    auto edge4 = GraphEdge(0, 2, 1);
    auto edge5 = GraphEdge(2, 5, 1);
    auto edge6 = GraphEdge(2, 6, 1);

    auto graph = Graph(7, std::vector<GraphEdge>{edge1, edge2, edge3, edge4, edge5, edge6});
    if (!graph.IsError()) {
        auto points = graph.FindArticulationPoint();
        graph.PrintPoints(points);
    }
    std::cout << "-------" << std::endl;
}

TEST_CASE("List test")
{
    auto edge1 = GraphEdge(0, 1, 1);
    auto edge2 = GraphEdge(1, 2, 1);
    auto edge3 = GraphEdge(2, 3, 1);
    auto edge4 = GraphEdge(3, 4, 1);
    auto edge5 = GraphEdge(4, 5, 1);
    auto edge6 = GraphEdge(5, 6, 1);
    auto graph = Graph(7, std::vector<GraphEdge>{edge1, edge2, edge3, edge4, edge5, edge6});
    if (!graph.IsError()) {
        auto points = graph.FindArticulationPoint();
        graph.PrintPoints(points);
    }
    std::cout << "-------" << std::endl;
}

TEST_CASE("Wheel graph test")
{
    auto edge1 = GraphEdge(0, 1, 1);
    auto edge2 = GraphEdge(1, 2, 1);
    auto edge3 = GraphEdge(2, 3, 1);
    auto edge4 = GraphEdge(3, 4, 1);
    auto edge5 = GraphEdge(4, 5, 1);
    auto edge6 = GraphEdge(5, 6, 1);
    auto edge7 = GraphEdge(6, 0, 1);
    auto graph = Graph(7, std::vector<GraphEdge>{edge1, edge2, edge3, edge4, edge5, edge6, edge7});
    if (!graph.IsError()) {
        auto points = graph.FindArticulationPoint();
        graph.PrintPoints(points);
    }
    std::cout << "-------" << std::endl;
}

TEST_CASE("Star graph test")
{
    auto edge1 = GraphEdge(0, 1, 1);
    auto edge2 = GraphEdge(0, 2, 1);
    auto edge3 = GraphEdge(0, 3, 1);
    auto edge4 = GraphEdge(0, 4, 1);
    auto edge5 = GraphEdge(0, 5, 1);
    auto edge6 = GraphEdge(0, 6, 1);
    auto graph = Graph(7, std::vector<GraphEdge>{edge1, edge2, edge3, edge4, edge5, edge6});
    if (!graph.IsError()) {
        auto points = graph.FindArticulationPoint();
        graph.PrintPoints(points);
    }
    std::cout << "-------" << std::endl;
}
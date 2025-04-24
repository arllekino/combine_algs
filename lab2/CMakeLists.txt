add_executable(graph graph.cpp index.cpp)
add_executable(graph_test graph.cpp test/test.cpp)

target_link_libraries(graph_test PRIVATE Catch2::Catch2WithMain)

include(Catch)
catch_discover_tests(graph_test)
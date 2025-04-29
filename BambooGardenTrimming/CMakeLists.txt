add_executable(bgt
        BGTNode/BGTNode.h
        BGTNode/BGTNode.cpp
        BGT/BambooGardenTrimming.h
        BGT/BambooGardenTrimming.cpp
        BGTCanvas/BGTCanvas.h
        BGTCanvas/BGTCanvas.cpp
        BGTController/BGTController.h
        BGTController/BGTController.cpp
        main.cpp)
#add_executable(bgt_test graph.cpp test/test.cpp)

#target_link_libraries(graph_test PRIVATE Catch2::Catch2WithMain)

include(Catch)
#catch_discover_tests(graph_test)
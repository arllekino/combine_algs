#include "./BGTController/BGTController.h"

std::unordered_set<BGTNode> nodes = {
    BGTNode(5, 5),
    BGTNode(10, 2),
    BGTNode(7, 3)
};

int main()
{
    auto bgtController = BGTController(nodes);



    return 0;
}

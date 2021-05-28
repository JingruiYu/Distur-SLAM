
#include <pangolin/pangolin.h>
#include <iostream>

int main(int argc, char const *argv[])
{
    pangolin::CreateWindowAndBind ( "visualize geometry", 1000, 600 );
    std::cout << "finish ... " << std::endl;
    return 0;
}

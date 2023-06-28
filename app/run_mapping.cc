#include "common/basetype.h"
#include "front_end/frontend.h"

string config_yaml = "./config/mapping.yaml";

int main(int argc, char** argv)
{
    cout << "app start" << endl;
    Frontend frontend(config_yaml);
    frontend.Init();
    frontend.Run();

    cout << "app end" << endl;
    while(1);
    return 0;
}
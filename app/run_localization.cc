#include "common/basetype.h"
#include "fusion/fusion.h"

string config_yaml = "./config/mapping.yaml";

int main(int argc, char** argv)
{
    cout << "app start" << endl;

    // sad::Fusion fusion(FLAGS_config_yaml);
    // if (!fusion.Init()) {
    //     return -1;
    // }
    Fusion fusion(config_yaml);
    fusion.Init();

    cout << "app end" << endl;
    while(1);
    return 0;
}
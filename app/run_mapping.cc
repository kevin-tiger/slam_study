#include "common/basetype.h"
#include "front_end/frontend.h"
#include "back_end/optimization.h"
#include "back_end/loopclosure.h"

string config_yaml = "./config/mapping.yaml";

int main(int argc, char** argv)
{
    cout << "app start" << endl;
    // Frontend frontend(config_yaml);
    // frontend.Init();
    // frontend.Run();

    // Optimization opti(config_yaml);
    // opti.Init(1);
    // opti.Run();

    // LoopClosure lc(config_yaml);
    // lc.Init();
    // lc.Run();

    Optimization opti2(config_yaml);
    opti2.Init(2);
    opti2.Run();

    cout << "app end" << endl;
    while(1);
    return 0;
}
#include "Application.h"

using namespace std;

int main(int argc, char *argv[]) {
    Application application(argv[1]);
    application.init();


    application.run("/home/age2pierre/OrboxWorkspace/Coins/Raw/", 0);


//    application.training();
//    application.testing();

//    Application.close();

    return 0;
}
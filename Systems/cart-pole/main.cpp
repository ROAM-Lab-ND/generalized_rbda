#include "CartPoleDynamics.hpp"
#include <unistd.h>

int main(int argc, char** argv) {
    CartPoleDynamics dyn;
    DVec<double> input(1);
    input.setZero();
    for(int i(0); i<100000; ++i){
        dyn.step(input);
        if(i%10 == 0){
          dyn.send_message();
        }
        usleep(1000);
    }

    return 0;
}

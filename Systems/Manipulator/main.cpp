#include "Openchain2DoF.hpp"
#include <unistd.h>

int main(int argc, char** argv) {
    Manipulator<double>* system = new Openchain2DoF();

    for(int i(0); i<10; ++i){
        if(!system->step()){ break; }
        if(i%10 == 0){
          system->send_message();
        }
        usleep(1000);
    }

    return 0;
}

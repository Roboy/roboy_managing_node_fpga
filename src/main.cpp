#include "roboy_managing_node_fpga/myoMaster.hpp"

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

    MyoMaster myoMaster;
    myoMaster.initialize(argc, argv);
    myoMaster.start();

}
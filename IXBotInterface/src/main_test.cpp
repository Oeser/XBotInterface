#include <iostream>
#include <XBotInterface/IXBotInterface.h>

////////////////////////////////////////////////////
// Main
////////////////////////////////////////////////////

int main ( int argc, char *argv[] ) {
    XBot::IXBotInterface& r = *XBot::IXBotInterface::getRobot(std::string(argv[1]));
} 


#ifndef COMTEST_H
#define COMTEST_H


#include "liborl/liborl.h"


namespace JcnsFranka {
    /**
     * @brief communication_test
     * Sends 10k empty commands and controls the response
     * @param ip name or ip address of the robot
     * @return number of lost states
     */
    uint64_t communication_test(char *ip);
}


#endif // COMTEST_H

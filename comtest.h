#ifndef COMTEST_H
#define COMTEST_H


#include "liborl/liborl.h"


namespace JcnsFranka {
    /**
     * @brief communicationTest
     * Method sends 10k empty commands to the Franka robot and checks the response
     * @return number of lost states
     */
    uint64_t communicationTest(char *ip);
}


#endif // COMTEST_H

#ifndef COMTEST_H
#define COMTEST_H


#include "liborl/liborl.h"


namespace JcnsFranka {
    /**
     * @brief communication_test
     * Method sends 10k empty commands and controls the response
     * @return number of lost states
     */
    uint64_t communication_test(char *ip);
}


#endif // COMTEST_H

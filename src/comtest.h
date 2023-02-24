#ifndef COMTEST_H
#define COMTEST_H


#include "liborl/liborl.h"


namespace JcnsFranka {
    /**
     * @brief communication_test
     * Sends 10k empty commands and controls the response
     * @param ip name or ip address of the robot
     * @param limit_rate whether rate limiters work or not
     * @param cutoff_frequency 1000 or low-pass filtered 100 [Hz]
     * @return number of lost states
     */
    uint64_t communication_test(char *ip, bool limit_rate,
                                double cutoff_frequency);
}


#endif // COMTEST_H

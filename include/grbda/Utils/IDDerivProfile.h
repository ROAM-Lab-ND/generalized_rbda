#ifndef GRBDA_ID_DERIV_PROFILE_H
#define GRBDA_ID_DERIV_PROFILE_H

#include <cstdint>

namespace grbda
{
namespace profiling
{

struct IDDerivCallProfile
{
    double forward_us = 0.0;
    double backward_us = 0.0;
    double casadi_us = 0.0;
    double getsq_us = 0.0;
    double getsq_internal_us = 0.0;
    std::uint64_t calls = 0;
};

bool isEnabled();
void resetCurrentCall();
void addCasadiUs(double us);
void addGetSqUs(double us);
void addGetSqInternalUs(double us);
void setForwardUs(double us);
void setBackwardUs(double us);
IDDerivCallProfile getCurrentCall();
void printCurrentCallIfEnabled();

} // namespace profiling
} // namespace grbda

#endif // GRBDA_ID_DERIV_PROFILE_H

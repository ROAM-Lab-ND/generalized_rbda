#include "grbda/Utils/IDDerivProfile.h"

#include <atomic>
#include <cstdlib>
#include <iomanip>
#include <iostream>

namespace grbda
{
namespace profiling
{
namespace
{
thread_local IDDerivCallProfile g_current_call;
std::atomic<std::uint64_t> g_print_count{0};

std::uint64_t maxPrints()
{
    const char *env = std::getenv("GRBDA_ID_DERIV_PROFILE_MAX_PRINTS");
    if (env == nullptr)
        return 1;

    const long long parsed = std::atoll(env);
    if (parsed <= 0)
        return 0;

    return static_cast<std::uint64_t>(parsed);
}

} // namespace

bool isEnabled()
{
    const char *env = std::getenv("GRBDA_ID_DERIV_PROFILE");
    return env != nullptr && env[0] != '0';
}

void resetCurrentCall()
{
    g_current_call = IDDerivCallProfile{};
    g_current_call.calls = 1;
}

void addCasadiUs(double us)
{
    g_current_call.casadi_us += us;
}

void addCasadiSUs(double us)
{
    g_current_call.casadi_s_us += us;
    g_current_call.casadi_us += us;
}

void addCasadiSRingUs(double us)
{
    g_current_call.casadi_s_ring_us += us;
    g_current_call.casadi_us += us;
}

void addCasadiSdotqdQUs(double us)
{
    g_current_call.casadi_sdotqd_q_us += us;
    g_current_call.casadi_us += us;
}

void addCasadiSdotqdQdUs(double us)
{
    g_current_call.casadi_sdotqd_qd_us += us;
    g_current_call.casadi_us += us;
}

void addGetSqUs(double us)
{
    g_current_call.getsq_us += us;
}

void addGetSqInternalUs(double us)
{
    g_current_call.getsq_internal_us += us;
}

void setForwardUs(double us)
{
    g_current_call.forward_us = us;
}

void setBackwardUs(double us)
{
    g_current_call.backward_us = us;
}

IDDerivCallProfile getCurrentCall()
{
    return g_current_call;
}

void printCurrentCallIfEnabled()
{
    if (!isEnabled())
        return;

    const std::uint64_t max_prints = maxPrints();
    const std::uint64_t idx = g_print_count.fetch_add(1);
    if (idx >= max_prints)
        return;

    const double total_us = g_current_call.forward_us + g_current_call.backward_us;
    const double other_us = total_us > g_current_call.casadi_us ? (total_us - g_current_call.casadi_us) : 0.0;

    std::cout << "[IDDerivProfile]"
              << " forward_us=" << std::fixed << std::setprecision(2) << g_current_call.forward_us
              << " backward_us=" << std::fixed << std::setprecision(2) << g_current_call.backward_us
              << " casadi_us=" << std::fixed << std::setprecision(2) << g_current_call.casadi_us
              << " casadi_s_us=" << std::fixed << std::setprecision(2) << g_current_call.casadi_s_us
              << " casadi_s_ring_us=" << std::fixed << std::setprecision(2) << g_current_call.casadi_s_ring_us
              << " casadi_sdotqd_q_us=" << std::fixed << std::setprecision(2) << g_current_call.casadi_sdotqd_q_us
              << " casadi_sdotqd_qd_us=" << std::fixed << std::setprecision(2) << g_current_call.casadi_sdotqd_qd_us
              << " getsq_us=" << std::fixed << std::setprecision(2) << g_current_call.getsq_us
              << " getsq_internal_us=" << std::fixed << std::setprecision(2) << g_current_call.getsq_internal_us
              << " other_us=" << std::fixed << std::setprecision(2) << other_us
              << " total_us=" << std::fixed << std::setprecision(2) << total_us
              << std::defaultfloat
              << std::endl;
}

} // namespace profiling
} // namespace grbda
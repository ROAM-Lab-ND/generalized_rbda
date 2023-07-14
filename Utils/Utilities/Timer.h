/*! @file Timer.h
 *  @brief Timer for measuring how long things take
 */

#ifndef GRBDA_TIMER_H
#define GRBDA_TIMER_H

#include <assert.h>
#include <stdint.h>
#include <time.h>

namespace grbda
{

  /*!
   * Timer for measuring time elapsed with clock_monotonic
   */
  class Timer
  {
  public:
    /*!
     * Construct and start timer
     */
    explicit Timer() { start(); }

    /*!
     * Start the timer
     */
    void start() { clock_gettime(CLOCK_MONOTONIC, &_startTime); }

    /*!
     * Get milliseconds elapsed
     */
    double getMs() { return (double)getNs() / 1.e6; }

    /*!
     * Get nanoseconds elapsed
     */
    int64_t getNs()
    {
      struct timespec now;
      clock_gettime(CLOCK_MONOTONIC, &now);
      return (int64_t)(now.tv_nsec - _startTime.tv_nsec) +
             1000000000 * (now.tv_sec - _startTime.tv_sec);
    }

    /*!
     * Get seconds elapsed
     */
    double getSeconds() { return (double)getNs() / 1.e9; }

    struct timespec _startTime;
  };

} // namespace grbda

#endif // PROJECT_TIMER_H

#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION__TIMING_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION__TIMING_H

#include <ros/ros.h>

#include <iostream>
#include <stack>
#include <unordered_map>
#include <chrono>

namespace whole_body_posture_optimization {
  #define START_TIMING(timerName) \
  Timing::startTiming(timerName);

  #define STOP_TIMING \
  Timing::stopTiming(false);

  #define STOP_TIMING_PRINT \
  Timing::stopTiming(true);

  #define STOP_TIMING_AVG \
  { \
  static int timing_timerId = -1; \
  Timing::stopTiming(false, timing_timerId); \
  }

  #define STOP_TIMING_AVG_PRINT \
  { \
  static int timing_timerId = -1; \
  Timing::stopTiming(true, timing_timerId); \
  }

#define INIT_TIMING \
  int whole_body_posture_optimization::IDFactory::id = 0; \
  std::unordered_map<int, whole_body_posture_optimization::AverageTime> whole_body_posture_optimization::Timing::m_averageTimes; \
  std::stack<whole_body_posture_optimization::TimingHelper> whole_body_posture_optimization::Timing::m_timingStack; \
  bool whole_body_posture_optimization::Timing::m_dontPrintTimes = false; \
  unsigned int whole_body_posture_optimization::Timing::m_startCounter = 0; \
  unsigned int whole_body_posture_optimization::Timing::m_stopCounter = 0;



  /** \brief Struct to store a time measurement.
  */
  struct TimingHelper
  {
    std::chrono::time_point<std::chrono::high_resolution_clock> start;
    std::string name;
  };

  /** \brief Struct to store the total time and the number of steps in order to compute the average time.
  */
  struct AverageTime
  {
    double totalTime;
    unsigned int counter;
    std::string name;
  };

  /** \brief Factory for unique ids.
  */
  class IDFactory
  {
  private:
    /** Current id */
    static int id;

  public:
    static int resetId() {id = 0;}
    static int getId() { return id++; }
  };

  /** \brief Class for time measurements.
  */
  class Timing
  {
  public:
    static bool m_dontPrintTimes;
    static unsigned int m_startCounter;
    static unsigned int m_stopCounter;
    static std::stack<TimingHelper> m_timingStack;
    static std::unordered_map<int, AverageTime> m_averageTimes;

    static void reset()
    {
      while (!m_timingStack.empty())
        m_timingStack.pop();
      m_averageTimes.clear();
      m_startCounter = 0;
      m_stopCounter = 0;
    }

    static void startTiming(const std::string& name = std::string(""))
    {
      TimingHelper h;
      h.start = std::chrono::high_resolution_clock::now();
      h.name = name;
      Timing::m_timingStack.push(h);
      Timing::m_startCounter++;
    }

    static double stopTiming(bool print = true)
    {
      if (!Timing::m_timingStack.empty())
      {
        Timing::m_stopCounter++;
        std::chrono::time_point<std::chrono::high_resolution_clock> stop = std::chrono::high_resolution_clock::now();
        TimingHelper h = Timing::m_timingStack.top();
        Timing::m_timingStack.pop();
        std::chrono::duration<double> elapsed_seconds = stop - h.start;
        double t = elapsed_seconds.count() * 1000.0;

        if (print)
          ROS_INFO_STREAM("time " << h.name.c_str() << ": " << t << " ms");
        return t;
      }
      return 0;
    }

    static double stopTiming(bool print, int &id)
    {
      if (id == -1)
        id = IDFactory::getId();
      if (!Timing::m_timingStack.empty())
      {
        Timing::m_stopCounter++;
        std::chrono::time_point<std::chrono::high_resolution_clock> stop = std::chrono::high_resolution_clock::now();
        TimingHelper h = Timing::m_timingStack.top();
        Timing::m_timingStack.pop();

        std::chrono::duration<double> elapsed_seconds = stop - h.start;
        double t = elapsed_seconds.count() * 1000.0;

        if (print && !Timing::m_dontPrintTimes)
          ROS_INFO_STREAM("time " << h.name.c_str() << ": " << t << " ms");

        if (id >= 0)
        {
          std::unordered_map<int, AverageTime>::iterator iter;
          iter = Timing::m_averageTimes.find(id);
          if (iter != Timing::m_averageTimes.end())
          {
            Timing::m_averageTimes[id].totalTime += t;
            Timing::m_averageTimes[id].counter++;
          }
          else
          {
            AverageTime at;
            at.counter = 1;
            at.totalTime = t;
            at.name = h.name;
            Timing::m_averageTimes[id] = at;
          }
        }
        return t;
      }
      return 0;
    }

    static void printAverageTimes()
    {
      std::unordered_map<int, AverageTime>::iterator iter;
      for (iter = Timing::m_averageTimes.begin(); iter != Timing::m_averageTimes.end(); iter++)
      {
        AverageTime &at = iter->second;
        const double avgTime = at.totalTime / at.counter;
        ROS_INFO_STREAM("Average time " << at.name.c_str() << ": " << avgTime << " ms");
      }
      if (Timing::m_startCounter != Timing::m_stopCounter)
        ROS_INFO_STREAM("Problem: " << Timing::m_startCounter << " calls of startTiming and " << Timing::m_stopCounter << " calls of stopTiming.");
      ROS_INFO_STREAM("---------------------------------------------------------------------------\n");
    }

    static void printTimeSums()
    {
      std::unordered_map<int, AverageTime>::iterator iter;
      for (iter = Timing::m_averageTimes.begin(); iter != Timing::m_averageTimes.end(); iter++)
      {
        AverageTime &at = iter->second;
        const double timeSum = at.totalTime;
        ROS_INFO_STREAM("Time sum " << at.name.c_str() << ": " << timeSum << " ms");
      }
      if (Timing::m_startCounter != Timing::m_stopCounter)
        ROS_INFO_STREAM("Problem: " << Timing::m_startCounter << " calls of startTiming and " << Timing::m_stopCounter << " calls of stopTiming. ");
        ROS_INFO_STREAM("---------------------------------------------------------------------------\n");
    }
  };
}

#endif

#ifndef MISSION_INCLUDED_H
#define MISSION_INCLUDED_H

#include <iostream>

  enum class MissionState {
         AWAIT_START,
         INIT,
         FIND_ENTRANCE,
         EXPLORE,
         STANDBY,
         STOP
   };

   std::ostream& operator<<(std::ostream& os, MissionState ms);

#endif 
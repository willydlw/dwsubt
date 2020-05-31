#include "base_control/mission.h"

std::ostream& operator<<(std::ostream& os, MissionState ms)
{
   switch(ms)
   {
      case MissionState::AWAIT_START:     os << "AWAIT_START";   break;
      case MissionState::EXPLORE:         os << "EXPLORE";       break;
      case MissionState::FIND_ENTRANCE:   os << "FIND_ENTRANCE"; break;
      case MissionState::INIT:            os << "INIT";          break;
      case MissionState::STANDBY:         os << "STANDBY";       break;
      case MissionState::STOP:            os << "STOP";          break;
   }

   return os;
}
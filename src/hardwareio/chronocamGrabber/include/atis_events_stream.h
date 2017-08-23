/*
 * atis_events_stream.h
 *
 *  Created on: Oct 20, 2016
 *      Author: sbrohan
 */

#ifndef UTILS_EXAMPLES_ATIS_EVENTS_STREAM_INC_ATIS_EVENTS_STREAM_H_
#define UTILS_EXAMPLES_ATIS_EVENTS_STREAM_INC_ATIS_EVENTS_STREAM_H_
#include <cstdint>
typedef long long timestamp;

typedef uint8_t Event_Types_underlying;

enum class Event_Types  : Event_Types_underlying {
    LEFT_TD_LOW     = 0x00,
    LEFT_TD_HIGH    = 0x01,
    LEFT_APS_START    = 0x02,
    LEFT_APS_END  = 0x03,
    RIGHT_TD_LOW    = 0x04,
    RIGHT_TD_HIGH   = 0x05,
    RIGHT_APS_END   = 0x06,
    RIGHT_APS_START = 0x07,
    EVT_TIME_HIGH   = 0x08,
    STEREO_DISP     = 0x09,
    ORIENTATION     = 0x0D,
    CONTINUED       = 0X0F

};



#ifdef _MSC_VER
#define PACK( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#elif defined(__GNUC__)
#define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))
#endif



PACK(
  struct EventBase {
    unsigned int trail:28;
    unsigned int type:4;
   });

PACK(
  struct Event_EVT_TIME_HIGH  {
    unsigned int timestamp:28;
    unsigned int type:4;
    });

PACK(
    struct Event_EVENT2D {
      unsigned int y:8;
      unsigned int x:9;
      unsigned int timestamp:11;
      unsigned int type:4;
    });

#endif /* UTILS_EXAMPLES_ATIS_EVENTS_STREAM_INC_ATIS_EVENTS_STREAM_H_ */

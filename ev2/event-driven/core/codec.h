/*
 *   Copyright (C) 2021 Event-driven Perception for Robotics
 *   Author: arren.glover@iit.it
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#pragma once
namespace ev {


typedef struct timeStamp {
    static const std::string tag;
#if ENABLE_TS
    unsigned int ts:31;
    unsigned int _fill:1;
#else
    static const unsigned int ts{0};
    static const unsigned int _fill{0};
#endif
} timeStamp;
using TS = timeStamp;

typedef struct addressEvent : public timeStamp {
    static const std::string tag;
    unsigned int p:1;
    unsigned int x:11;
    unsigned int y:10;
    unsigned int channel:1;
    unsigned int type:1;
    unsigned int skin:1;
    unsigned int corner:1;
    unsigned int _fill:6;
} addressEvent;
using AE = addressEvent;

typedef struct encoded : public timeStamp {
    static const std::string tag;
    int32_t data;
} encoded;

typedef struct skinAE : public timeStamp {
    static const std::string tag;
    unsigned int polarity:1;
    unsigned int taxel:4;
    unsigned int device:4;
    unsigned int constant:23;
} skinAE;

typedef struct skinValue {
    unsigned int _ts:31;
    unsigned int _fill1:1;
    unsigned int value:16;
    unsigned int _fill2:16;
} skinValue;

typedef struct skinSample {
    static const std::string tag;
    skinAE address;
    skinValue value;
} skinSample;

/// \brief an AddressEvent with a velocity in visual space
typedef struct flowEvent : public AE {
    static const std::string tag;
    float vx;
    float vy;
} flowEvent;

/// \brief a LabelledAE with parameters that define a 2D gaussian
typedef struct gaussianEvent {
    static const std::string tag;
    float sigx;
    float sigy;
    float sigxy;
} gaussianEvent;

/// \brief an event with a pixel location, camera number and polarity
typedef struct IMUS : public timeStamp {
    static const std::string tag;
    int value:16;
    unsigned int sensor:4;
    unsigned int _r1:2;
    unsigned int channel:1;
    unsigned int type:1;
    unsigned int _r2:8;
} IMUS;

typedef struct neuronEvent : public timeStamp {
    static const std::string tag;
    int id;
} neuronEvent;

typedef struct earEvent : public timeStamp {
    static const std::string tag;
    unsigned int polarity : 1;
    unsigned int freq_chnn : 7;
    unsigned int xso_type : 1;
    unsigned int auditory_model : 1;
    unsigned int _reserved1 : 2;
    unsigned int neuron_id : 7;
    unsigned int sensor_id : 3;
    unsigned int channel : 1;
    unsigned int type : 1;
    unsigned int cochlea_sensor_id : 3;
    unsigned int _fill : 5;
} earEvent;

}
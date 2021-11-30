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
    unsigned int ts:31;
    unsigned int _fill:1;
} timeStamp;
using TS = timeStamp;

typedef struct addressEvent {
    static const std::string tag;
    unsigned int p:1;
    unsigned int x:10;
    unsigned int _xfill:1;
    unsigned int y:9;
    unsigned int corner:1;
    unsigned int channel:1;
    unsigned int type:1;
    unsigned int skin:1;
    unsigned int _fill:7;
} addressEvent;
using AE = addressEvent;

typedef struct TAE : public timeStamp, public addressEvent {
    static const std::string tag;
} TAE;


typedef struct skinAE : public timeStamp {
    static const std::string tag;
    unsigned int polarity:1;
    unsigned int taxel:10;
    unsigned int _reserved1:2;
    unsigned int cross_base:1;
    unsigned int _sample:1;
    unsigned int _error:1;
    unsigned int body_part:3;
    unsigned int _reserved2:3;
    unsigned int side:1;
    unsigned int type:1;
    unsigned int skin:1;
    unsigned int _fill:7;
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
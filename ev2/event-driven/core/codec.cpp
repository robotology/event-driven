/*
 *   Copyright (C) 2017 Event-driven Perception for Robotics
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
#include <string>
#include <event-driven/core/codec.h>

const std::string ev::timeStamp::tag = "TS";
const std::string ev::addressEvent::tag = "AE";
const std::string ev::encoded::tag="AE";
const std::string ev::skinAE::tag = "AE";
const std::string ev::skinSample::tag = "SKS";
const std::string ev::flowEvent::tag = "FLOW";
const std::string ev::gaussianEvent::tag = "GAE";
const std::string ev::IMUS::tag = "IMU";
const std::string ev::neuronEvent::tag = "NEU";
const std::string ev::earEvent::tag = "EAR";
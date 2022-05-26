/*
 *   Copyright (C) 2020 Event-driven Perception for Robotics
 *   Author: dgutierrez@atc.us.es
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

#include <yarp/os/Bottle.h>
#include "event-driven/vCodec.h"

namespace ev {

    const std::string CochleaEvent::tag = "EAR";

    CochleaEvent::CochleaEvent() : vEvent(), polarity(0), freq_chnn(0), xso_type(0), auditory_model(0),
                                   _reserved1(0), neuron_id(0), sensor_id(0), channel(0), type(0),
                                   cochlea_sensor_id(0), _fill(0) {}

    CochleaEvent::CochleaEvent(const vEvent &v) : vEvent(v), _cochleaei(0) {
        const CochleaEvent *v2 = dynamic_cast<const CochleaEvent *>(&v);
        if (v2) {
            _cochleaei = v2->_cochleaei;
        }
    }

    CochleaEvent::CochleaEvent(const CochleaEvent &v) : vEvent(v) {
        _cochleaei = v._cochleaei;
    }

    event<> CochleaEvent::clone() {
        return std::make_shared<CochleaEvent>(*this);
    }

    void CochleaEvent::encode(yarp::os::Bottle &b) const {
        vEvent::encode(b);
        b.addInt32(_cochleaei);
    }

    void CochleaEvent::encode(std::vector<int32_t> &b, unsigned int &pos) const {
        vEvent::encode(b, pos);
        b[pos++] = _cochleaei;
    }

    void CochleaEvent::decode(const int32_t *&data) {
        vEvent::decode(data);
        _cochleaei = *(data++);
    }

    bool CochleaEvent::decode(const yarp::os::Bottle &packet, size_t &pos) {
        // check length
        if (vEvent::decode(packet, pos) && pos + 1 <= packet.size()) {
            _cochleaei = packet.get(pos++).asInt32();
            return true;
        }
        return false;
    }

    yarp::os::Property CochleaEvent::getContent() const {
        yarp::os::Property prop = vEvent::getContent();
        prop.put("auditory_model", (int) auditory_model);
        prop.put("channel", (int) channel);
        prop.put("xso_type", (int) xso_type);
        prop.put("neuron_id", (int) neuron_id);
        prop.put("freq_chnn", (int) freq_chnn);
        prop.put("polarity", (int) polarity);
        prop.put("sensor_id", (int) sensor_id);
        prop.put("type", (int) type);

        return prop;
    }

    std::string CochleaEvent::getType() const {
        return CochleaEvent::tag;
    }

    int CochleaEvent::getChannel() const {
        return (int) channel;
    }

    void CochleaEvent::setChannel(const int channel) {
        this->channel = channel;
    }
    
}
// namespace ev

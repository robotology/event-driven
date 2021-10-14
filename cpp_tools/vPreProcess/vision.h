/*
 *   Copyright (C) 2017 Event-driven Perception for Robotics
 *   Author: arren.glover@iit.it
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once
#include <event-driven/core.h>
#include <yarp/os/all.h>
#include <string>

class visionFunctions 
{
private:

    //splitting options
    bool output_stereo{false};
    bool output_polarities{false};
    bool output_corners{false};
    
    //processing - flipping
    ev::resolution res{640, 480};
    bool flipx{false};
    bool flipy{false};

    //processing - undistort/rectify
    bool undistort{false};
    ev::vIPT calibrator;

    //processing - filter
    bool apply_filter{false};
    ev::vNoiseFilter filter_left;
    ev::vNoiseFilter filter_right;
    int v_total{0};
    int v_dropped{0};
    
    //ports and packets
    bool opened{false};
    enum port_label { LEFT, RIGHT, LNEG, RNEG, LCOR, RCOR, STEREO};
    ev::BufferedPort<ev::AE> ports[7];
    ev::packet<ev::AE> *packets[7] = 
        {nullptr,nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};

public:

    void stats(int &passed, int &dropped)
    {
        passed = v_total;
        dropped = v_dropped;
        v_total = 0;
        v_dropped = 0;
    }

    void init_flips(bool x, bool y, ev::resolution r)
    {
        res = r;
        yInfo() << "[VISION]: camera size" << r.width << "x" << r.height;
        flipx = x;
        if(flipx) yInfo() << "[VISION]: flip horizontal";
        flipy = y;
        if(flipy) yInfo() << "[VISION]: flip vertical";
    }

    void init_splits(bool stereo = false, bool polarities = false, bool corners = false) 
    {
        output_stereo = stereo;
        if(output_stereo) yInfo() << "[VISION]: output stereo events";
        output_polarities = polarities;
        if(output_polarities) yInfo() << "[VISION]: splitting polarities";
        output_corners = corners;
        if(output_corners) yInfo() << "[VISION]: output corner seperately";
    }

    void init_filter(double T_temporal = 0, double T_spatial = 0) 
    {
        if(T_temporal > 0.0 || T_spatial > 0.0) {
            filter_left.initialise(res.width, res.height);
            filter_right.initialise(res.width, res.height);
            apply_filter = true;
        }

        if (T_temporal > 0.0) {
            filter_left.use_temporal_filter(ev::secondsToTicks(T_temporal));
            filter_right.use_temporal_filter(ev::secondsToTicks(T_temporal));
            yInfo() << "[VISION]: refractory filter - " << T_temporal << "secs";
        }

        if (T_spatial > 0.0) {
            filter_left.use_spatial_filter(ev::secondsToTicks(T_spatial));
            filter_right.use_spatial_filter(ev::secondsToTicks(T_spatial));
            yInfo() << "[VISION]: pepper filter - " << T_temporal << "secs";
        }
    }

    void init_undistort(std::string calibration_file_path) 
    {
        if (calibrator.configure(calibration_file_path)) {
            yInfo() << "[VISION]: undistort image";
            calibrator.printValidCalibrationValues();
            undistort = true;
        } else {
            yError() << "Could not correctly configure the cameras";
        }
    }

    bool _openPort(const port_label label, const std::string name)
    {
        if (!ports[label].open(name)) {
            yError() << "Could not open" << name;
            return false;
        }
        packets[label] = &(ports[label].prepare());
        return true;
    }

    bool open(std::string mname)
    {
        if (output_stereo)
            if(!_openPort(STEREO, mname + "/stereo/AE:o"))
                return false;
        
        if (output_polarities) {
            if(!_openPort(LNEG, mname + "/left/neg/AE:o"))
                return false;
            if(!_openPort(RNEG, mname + "/right/neg/AE:o"))
                return false;
            if (!_openPort(LEFT, mname + "/left/pos/AE:o"))
                return false;
            if (!_openPort(RIGHT, mname + "/right/pos/AE:o"))
                return false;
        }

        if (output_corners) {
            if (!_openPort(LCOR, mname + "/left/corner/AE:o"))
                return false;
            if (!_openPort(RCOR, mname + "/right/corner/AE:o"))
                return false;
        }

        if (!output_polarities) {
            if (!_openPort(LEFT, mname + "/left/AE:o"))
                return false;
            if (!_openPort(RIGHT, mname + "/right/AE:o"))
                return false;
        }

        opened = true;
        return true;
    }

    void process(ev::AE *datum)
    {
        if(!opened) return;
        //flipping
        if (flipx) datum->x = res.width  - datum->x - 1;
        if (flipy) datum->y = res.height - datum->y - 1;

        //salt-n-pepper filter
        if (apply_filter) {
            if (datum->channel == ev::CAMERA_LEFT) {
                if (!filter_left.check(datum->x, datum->y, datum->p, datum->ts)) {
                    v_dropped++;
                    return;
                }
            } else {
                if (!filter_right.check(datum->x, datum->y, datum->p, datum->ts)) {
                    v_dropped++;
                    return;
                }
            }
        }

        v_total++;

        //undistortion and rectification
        if (undistort) {
            int y = datum->y; int x = datum->x;
            calibrator.sparseForwardTransform(datum->channel, y, x);
            datum->y = y; datum->x = x;
        }

        //output to stereo combined stream
        if (output_stereo) packets[STEREO]->push_back(*datum);

        //output to corners stream
        if (output_corners && datum->corner) {
            if(datum->channel == ev::CAMERA_LEFT)
                packets[LCOR]->push_back(*datum);
            else
                packets[RCOR]->push_back(*datum);
        }

        //output stereo split streams (splitting also by polarity if needed)
        if (output_polarities && datum->p == 0) {
            if (datum->channel == ev::CAMERA_LEFT)
                packets[LNEG]->push_back(*datum);
            else
                packets[RNEG]->push_back(*datum);
        } else {
            if (datum->channel == ev::CAMERA_LEFT)
                packets[LEFT]->push_back(*datum);
            else
                packets[RIGHT]->push_back(*datum);
        }
    }

    void send(yarp::os::Stamp stamp, double duration)
    {
        for(int pl = LEFT; pl <= STEREO; pl++) {
            if(packets[pl] && packets[pl]->size()) {
                packets[pl]->duration(duration);
                ports[pl].setEnvelope(stamp);
                ports[pl].write();
                packets[pl] = &(ports[pl].prepare());
            }
        }
    }

    void close()
    {
        for(int pl = LEFT; pl <= STEREO; pl++) {
            packets[pl] = nullptr;
            ports[pl].unprepare();
            ports[pl].close();
        }
    }

};
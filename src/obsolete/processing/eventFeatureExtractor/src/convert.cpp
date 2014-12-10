// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Rea Francesco, Charles Clercq
 * email:   francesco.rea@iit.it, charles.clercq@iit.it
 * website: www.robotcub.org 
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

/**
 * @file convert.cpp
 * @brief A class for conversion(see header convert.h)
 */

#include <iCub/convert.h>
using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

converter::converter(int i_width, int i_height) {
    cout << "C_printingThread::C_printingThread(...)" << endl;
    base_img.resize(i_width, i_height);
    cart_pol_acc = new int*[i_width];
    for(int i=0; i<i_width; i++) {
        cart_pol_acc[i] = new int[i_height];
        for(int j=0; j<i_height; j++){
            cart_pol_acc[i][j] = 0;
            base_img(i,j) = 125;
        }
    }
    height = i_height;
    width = i_width;
    //port.open("/image/cartesian:o");
}

converter::~converter(){
    for(int i=0; i<width; i++)
        delete[] cart_pol_acc[i];
    delete[] cart_pol_acc;
    port.close();
}

ImageOf<PixelMono16> converter::create_frame(list<AER_struct> i_lAER) {
    int size_stack;
    int _x;
    int _y;
    int _pol;
    double _ts;

    cout << "Initialisation of the table of accumulation of the events" << endl;
    start = clock();
    while(( ((double)start/CLOCKS_PER_SEC)*1e3+20.0) > (((double)clock()/CLOCKS_PER_SEC)*1e3))
    {
    //             cout << ((double)start/CLOCKS_PER_SEC)*1e3+200.0 << "\t" << ((double)clock()/CLOCKS_PER_SEC)*1e3 << endl;
    }
    cout << "Size of the list (number of event packed) : " << i_lAER.size() << endl;
    size_stack = (int)i_lAER.size();

    //Init the base-frame in gray value
    ImageOf<PixelMono16> _tmp_cart_img = base_img;

    if(size_stack > 0) {
        for(int i=0; i<width; i++) {
            for(int j=0; j<height; j++)
                cart_pol_acc[i][j] = 0;
        }
        for(int index=0;index<size_stack;index++) {
            AER_struct event = i_lAER.front();
            _x = event.y;//event.x;
            _y = event.x;//event.y;
            _pol = event.pol;
            _ts = event.ts;
            i_lAER.pop_front();
//            cout << '\t' << _x << " " << _y << " " << _pol << " " << _ts << endl;

            if(_x!=-1 && _y!=-1) {
                //**Cartesian image computation
                cart_pol_acc[_x][_y]+=_pol;
                if(cart_pol_acc[_x][_y]< 0)
                    _tmp_cart_img(127-_y,127-_x)=0;//_tmp_cart_img(_x,_y)=0;
                else
                    if(cart_pol_acc[_x][_y]> 0)
                        _tmp_cart_img(127-_y,127-_x)=255;//_tmp_cart_img(_x,_y)=255;
                    else
                        _tmp_cart_img(127-_y,127-_x)=125;//_tmp_cart_img(_x,_y)=125;
                //******************************
            }
        }
    }
    return _tmp_cart_img;
}

void converter::send_frame(ImageOf<PixelMono16> i_img) {
    ImageOf<PixelMono16>& tmp = port.prepare();
    tmp = i_img;
    port.write();
}

int converter::sign(int i_val) {
    if(i_val > 0)
        return 1;
    else if(i_val < 0)
        return -1;
    else
        return 0;
}

float converter::mean_event(int i_nE) {
    static float number_of_data = 0.0;
    static float mean_time_loop = 0.0;
    number_of_data++;
    if(number_of_data == 1)
        mean_time_loop = (float)i_nE;
    else
        mean_time_loop = mean_time_loop + (1/number_of_data)*((float)i_nE-mean_time_loop);
    return mean_time_loop;
}

//----- end-of-file --- ( next line intentionally left blank ) ------------------


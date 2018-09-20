/*
 *   Copyright (C) 2017 Event-driven Perception for Robotics
 *   Author: arren.glover@iit.it
 *           valentina.vasco@iit.it
 *           chiara.bartolozzi@iit.it
 *           massimiliano.iacono@iit.it
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

#include "vDraw.h"
//#define _USE_MATH_DEFINES
//#include <math.h>

using namespace ev;

vDraw * createDrawer(std::string tag)
{

    if(tag == addressDraw::drawtype)
        return new addressDraw();
    if(tag == isoDraw::drawtype)
        return new isoDraw();
    if(tag == interestDraw::drawtype)
        return new interestDraw();
    if(tag == circleDraw::drawtype)
        return new circleDraw();
    if(tag == flowDraw::drawtype)
        return new flowDraw();
    if(tag == lifeDraw::drawtype)
        return new lifeDraw();
    if(tag == clusterDraw::drawtype)
        return new clusterDraw();
    if(tag == blobDraw::drawtype)
        return new blobDraw();
    if(tag == skinDraw::drawtype)
        return new skinDraw();
    if(tag == accDraw::drawtype)
        return new accDraw();
    if(tag == isoInterestDraw::drawtype)
        return new isoInterestDraw();
    if(tag == isoCircDraw::drawtype)
        return new isoCircDraw();
    return 0;

}

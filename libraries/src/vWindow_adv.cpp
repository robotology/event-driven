#include "iCub/eventdriven/vWindow_adv.h"
#include <math.h>

namespace ev {

vSurface2::vSurface2(int width, int height)
{
    this->width = width;
    this->height = height;
    this->count = 0;

    spatial.resize(height);
    for(int y = 0; y < height; y++) {
        spatial[y].resize(width);
    }
}

void vSurface2::fastAddEvent(event <> v, bool onlyAdd)
{
    auto c = as_event<AE>(v);
    if(c->y >= height || c->x >= width) {
        return;
    }

    if(!onlyAdd)
        fastRemoveEvents(v);

    q.push_back(v);

    if(!spatial[c->y][c->x])
        count++;

    spatial[c->y][c->x] = c;

    return;


}

vQueue vSurface2::addEvent(event<> v)
{
    auto c = is_event<AE>(v);
    if(c->y >= height || c->x >= width) {
        //std::cout << "WHY" << std::endl;
        return vQueue();
    }

    vQueue removed = removeEvents(v);

    q.push_back(v);
    if(c) {

        if(spatial[c->y][c->x])
            removed.push_back(spatial[c->y][c->x]);
        else
            count++;

        spatial[c->y][c->x] = c;
    }

    return removed;

}

vQueue vSurface2::getSurf()
{
    return getSurf(0, width, 0, height);
}

vQueue vSurface2::getSurf(int d)
{
    event<AddressEvent> v(nullptr);
    for(vQueue::reverse_iterator qi = q.rbegin(); qi != q.rend(); qi++) {
        v = as_event<AddressEvent>(*qi);
        if(v) break;
    }
    if(!v) return vQueue();

    return getSurf(v->x, v->y, d);

}

vQueue vSurface2::getSurf(int x, int y, int d)
{
    return getSurf(x - d, x + d, y - d, y + d);
}

vQueue vSurface2::getSurf(int xl, int xh, int yl, int yh)
{
    vQueue qcopy;

    xl = std::max(xl, 0);
    xh = std::min(xh, width-1);
    yl = std::max(yl, 0);
    yh = std::min(yh, height-1);

    for(int y = yl; y <= yh; y++)
        for(int x = xl; x <= xh; x++)
            if(spatial[y][x]) qcopy.push_back(spatial[y][x]);

    return qcopy;

}

void vSurface2::getSurfSorted(vQueue &fillq)
{
    fillq.resize(count);
    if(!count) return;

    unsigned int i = 0;
    vQueue::reverse_iterator rqit;
    for(rqit = q.rbegin(); rqit != q.rend(); rqit++) {
        event<AddressEvent> v = std::static_pointer_cast<AddressEvent>(*rqit);
        if(v != spatial[v->y][v->x]) continue;
        fillq[i++] = v;
    }
}

vQueue vSurface2::getSurf_Tlim(int dt)
{
    vQueue qcopy;
    if(q.empty()) return qcopy;

    int t = q.back()->stamp;

    for(vQueue::reverse_iterator rqit = q.rbegin(); rqit != q.rend(); rqit++) {

        //check it is on the surface
        event<AddressEvent> v = std::static_pointer_cast<AddressEvent>(*rqit);
        if(v != spatial[v->y][v->x]) continue;

        //check temporal constraint
        int vt = (*rqit)->stamp;
        if(vt > t) vt -= vtsHelper::max_stamp;
        if(vt + dt <= t) break;

        qcopy.push_back(v);
    }

    return qcopy;
}

vQueue vSurface2::getSurf_Tlim(int dt, int d)
{
    event<AddressEvent> v(nullptr);
    for(vQueue::reverse_iterator qi = q.rbegin(); qi != q.rend(); qi++) {
        v = as_event<AddressEvent>(*qi);
        if(v) break;
    }
    if(!v) return vQueue();

    return getSurf_Tlim(dt, v->x, v->y, d);

}

vQueue vSurface2::getSurf_Tlim(int dt, int x, int y, int d)
{
    return getSurf_Tlim(dt, x - d, x + d, y - d, y + d);

}

vQueue vSurface2::getSurf_Tlim(int dt, int xl, int xh, int yl, int yh)
{

    vQueue qcopy;
    if(q.empty()) return qcopy;

    int t = q.back()->stamp;

    for(vQueue::reverse_iterator rqit = q.rbegin(); rqit != q.rend(); rqit++) {

        //check it is on the surface
        event<AddressEvent> v = std::static_pointer_cast<AddressEvent>(*rqit);
        if(v != spatial[v->y][v->x]) continue;

        //check temporal constraint
        int vt = (*rqit)->stamp;
        if(vt > t) vt -= vtsHelper::max_stamp;
        if(vt + dt <= t) break;

        //check spatial constraint
        if(v->x >= xl && v->x <= xh) {
            if(v->y >= yl && v->y <= yh) {
                qcopy.push_back(v);
            }
        }
    }

    return qcopy;
}

vQueue vSurface2::getSurf_Clim(int c)
{
    return getSurf_Clim(c, 0, width, 0, height);
}

vQueue vSurface2::getSurf_Clim(int c, int d)
{
    event<AddressEvent> v(nullptr);
    for(vQueue::reverse_iterator qi = q.rbegin(); qi != q.rend(); qi++) {
        v = as_event<AddressEvent>(*qi);
        if(v) break;
    }
    if(!v) return vQueue();

    return getSurf_Clim(c, v->x, v->y, d);

}

vQueue vSurface2::getSurf_Clim(int c, int x, int y, int d)
{
    return getSurf_Clim(c, x - d, x + d, y - d, y + d);

}

vQueue vSurface2::getSurf_Clim(int c, int xl, int xh, int yl, int yh)
{
    vQueue qcopy;
    if(q.empty()) return qcopy;

    xl = std::max(xl, 0);
    xh = std::min(xh, width-1);
    yl = std::max(yl, 0);
    yh = std::min(yh, height-1);

    for(int y = yl; y <= yh; y++) {
        for(int x = xl; x <= xh; x++) {
            if(spatial[y][x]) qcopy.push_back(spatial[y][x]);
        }
    }

    qsort(qcopy, true);
    while(qcopy.size() > (unsigned int)c)
        qcopy.pop_front();

    return qcopy;
}


event<> vSurface2::getMostRecent()
{
    if(!q.size()) return NULL;
    return q.back();
}

/******************************************************************************/
vQueue temporalSurface::removeEvents(event<> toAdd)
{
    vQueue removed;

    //calculate event window boundaries based on latest timestamp
    int ctime = toAdd->stamp;
    int upper = ctime + vtsHelper::max_stamp - duration;
    int lower = ctime - duration;

    //remove any events falling out the back of the window
    while(q.size()) {

        event<AddressEvent> v = std::static_pointer_cast<AddressEvent>(q.front());
        if(v && v != spatial[v->y][v->x]) {
            q.pop_front();
            continue;
        }

        int vtime = q.front()->stamp;

        if((vtime > ctime && vtime < upper) || vtime < lower) {
            removed.push_back(q.front());
            if(v) spatial[v->y][v->x] = NULL;
            q.pop_front();
            count--;
        } else {
            break;
        }
    }

    while(q.size()) {

        event<AddressEvent> v = std::static_pointer_cast<AddressEvent>(q.back());
        if(v && v != spatial[v->y][v->x]) {
            q.pop_back();
            continue;
        }

        int vtime = q.back()->stamp;

        if((vtime > ctime && vtime < upper) || vtime < lower) {
            removed.push_back(q.back());
            if(v) spatial[v->y][v->x] = NULL;
            q.pop_back();
            count--;
        } else {
            break;
        }
    }

    return removed;
}

/******************************************************************************/
void temporalSurface::fastRemoveEvents(event<> toAdd)
{

    //calculate event window boundaries based on latest timestamp
    int ctime = toAdd->stamp;
    int upper = ctime + vtsHelper::max_stamp - duration;
    int lower = ctime - duration;

    //remove any events falling out the back of the window
    while(q.size()) {

        event<AddressEvent> v = std::static_pointer_cast<AddressEvent>(q.front());
        if(v && v != spatial[v->y][v->x]) {
            q.pop_front();
            continue;
        }

        int vtime = v->stamp;

        if((vtime > ctime && vtime < upper) || vtime < lower) {
            if(v) spatial[v->y][v->x] = NULL;
            q.pop_front();
            count--;
        } else {
            break;
        }
    }

//    while(q.size()) {

//        event<AddressEvent> v = std::static_pointer_cast<AddressEvent>(q.back());
//        if(v && v != spatial[v->y][v->x]) {
//            q.pop_back();
//            continue;
//        }

//        int vtime = v->stamp;

//        if((vtime > ctime && vtime < upper) || vtime < lower) {
//            if(v) spatial[v->y][v->x] = NULL;
//            q.pop_back();
//            count--;
//        } else {
//            break;
//        }
//    }
}

/******************************************************************************/
vQueue fixedSurface::removeEvents(event<> toAdd)
{

    vQueue removed;

    while(q.size()) {

        event<AddressEvent> v = as_event<AddressEvent>(q.front());
        if(v && v != spatial[v->y][v->x]) {
            q.pop_front();
        } else {
            break;
        }
    }

    if(count > qlength) {

        removed.push_back(q.front());
        event<AddressEvent> v = as_event<AddressEvent>(q.front());
        if(v) spatial[v->y][v->x] = NULL;
        q.pop_front();
        count--;
    }

    return removed;

}

void fixedSurface::fastRemoveEvents(event<> toAdd)
{

    while(q.size()) {

        event<AddressEvent> v = as_event<AddressEvent>(q.front());
        if(v && v != spatial[v->y][v->x]) {
            q.pop_front();
        } else {
            break;
        }
    }

    if(count > qlength) {

        event<AddressEvent> v = as_event<AddressEvent>(q.front());
        if(v) spatial[v->y][v->x] = NULL;
        q.pop_front();
        count--;
    }

}

/******************************************************************************/
vQueue lifetimeSurface::addEvent(event<> toAdd)
{

    event<FlowEvent> v = as_event<FlowEvent>(toAdd);
    if(!v) return vQueue();
    return vSurface2::addEvent(v);
}

vQueue lifetimeSurface::removeEvents(event<> toAdd)
{

    vQueue removed;
    //this could be better if q is sorted by death.


    //lifetime requires a flow event only
    event<FlowEvent> toAddflow = as_event<FlowEvent>(toAdd);
    if(!toAddflow)
        return vQueue();

    int cts = toAddflow->stamp;
    int cx = toAddflow->x; int cy = toAddflow->y;


    vQueue::iterator i = q.begin();
    while(i != q.end()) {
        event<FlowEvent> v = std::static_pointer_cast<FlowEvent>(*i);
        int modts = cts;
        if(cts < v->stamp) //we have wrapped
            modts += vtsHelper::max_stamp;

        bool samelocation = v->x == cx && v->y == cy;

        if(modts > v->getDeath() || samelocation) {
            //it could be dangerous if spatial gets more than 1 event per pixel
            removed.push_back(*i);
            spatial[v->y][v->x] = NULL;
            i = q.erase(i);
            count--;
        } else {
            i++;
        }
    }

    return removed;

}

void lifetimeSurface::fastRemoveEvents(event<> toAdd)
{

    //lifetime requires a flow event only
    event<FlowEvent> toAddflow = as_event<FlowEvent>(toAdd);
    if(!toAddflow)
        return;

    int cts = toAddflow->stamp;
    int cx = toAddflow->x; int cy = toAddflow->y;


    vQueue::iterator i = q.begin();
    while(i != q.end()) {
        event<FlowEvent> v = std::static_pointer_cast<FlowEvent>(*i);
        int modts = cts;
        if(cts < v->stamp) //we have wrapped
            modts += vtsHelper::max_stamp;

        bool samelocation = v->x == cx && v->y == cy;

        if(modts > v->getDeath() || samelocation) {
            //it could be dangerous if spatial gets more than 1 event per pixel
            spatial[v->y][v->x] = NULL;
            i = q.erase(i);
            count--;
        } else {
            i++;
        }
    }

}

bool vEdge::flowremove(vQueue &removed, event<FlowEvent> vf)
{

    int x = vf->x; int y = vf->y;
    double vx = vf->vy; double vy = vf->vx;
    double vmag = sqrt(pow(vx, 2.0) + pow(vy, 2.0));
    vx = vx / vmag; vy = vy / vmag;
    int px, py;

    //remove event at this location
    if(spatial[y][x]) {
        removed.push_back(spatial[y][x]);
        spatial[y][x] = event<AE>(nullptr);
    }

    double a = 0.5;
    double t = 0.8;//0.708;
    int f = 3;
    double vx1 = vx + a * vy; double vy1 = vy - a * vx;
    double vx2 = vx - a * vy; double vy2 = vy + a * vx;

    //remove events not on the perpendicular line
    for(int yi = -f; yi <= f; yi++) {
        for(int xi = -f; xi <= f; xi++) {
            double d1 = xi * vx1 + yi * vy1;
            double d2 = xi * vx2 + yi * vy2;
            if(fabs(d1) > t && fabs(d2) > t && d1 * d2 >= 0) {
                px = x + xi; py = y + yi;
                if(px >= 0 && py >= 0 && px < res.width && py < res.height) {
                    if(spatial[py][px]) {
                        removed.push_back(spatial[py][px]);
                        spatial[py][px] = event<AE>(nullptr);
                    }
                }
            }
        }
    }

    return true;

}

bool vEdge::addressremove(vQueue &removed, event<AddressEvent> v)
{

    int x = v->x; int y = v->y;
    if(spatial[y][x]) {
        removed.push_back(spatial[y][x]);
        spatial[y][x] = event<AE>(nullptr);
    }


    for(int yi = -1 + y; yi <= 1 + y; yi++) {
        for(int xi = -1 + x; xi <= 1 + x; xi++) {

            if(yi < 0 || xi < 0 || yi >= res.height || xi >= res.width) continue;
            if(spatial[yi][xi] && pepperCheck(yi, xi)) {
                removed.push_back(spatial[yi][xi]);
                spatial[yi][xi] = event<AE>(nullptr);;
            }
        }
    }

    return !pepperCheck(y, x);


}

bool vEdge::pepperCheck(int y, int x)
{
    if(y == 0 || y == res.height-1 || x == 0 || x == res.width -1)
        return true;

    if(spatial[y-1][x] && as_event<FlowEvent>(spatial[y-1][x]))
        return false;
    if(spatial[y+1][x] && as_event<FlowEvent>(spatial[y+1][x]))
        return false;
    if(spatial[y][x-1] && as_event<FlowEvent>(spatial[y][x-1]))
        return false;
    if(spatial[y][x+1] && as_event<FlowEvent>(spatial[y][x+1]))
        return false;
    if(spatial[y+1][x+1] && as_event<FlowEvent>(spatial[y+1][x+1]))
        return false;
    if(spatial[y-1][x+1] && as_event<FlowEvent>(spatial[y-1][x+1]))
        return false;
    if(spatial[y+1][x-1] && as_event<FlowEvent>(spatial[y+1][x-1]))
        return false;
    if(spatial[y-1][x-1] && as_event<FlowEvent>(spatial[y-1][x-1]))
        return false;

    return true;
}



vQueue vEdge::addEventToEdge(event<AddressEvent> v)
{
    vQueue removed;
    bool add = false;

    if(!v) return removed;

    //remove non edge events
    event<FlowEvent> vf = as_event<FlowEvent>(v);
    if(!vf) {
        addressremove(removed, v);
    } else {
        add = flowremove(removed, vf);
    }


    if(trackCount) {
        if(add) eventCount++;
        eventCount -= removed.size();
    }

    if(add) {
        //put our new event in and add a reference
        spatial[v->y][v->x] = v;

        //then set our mostRecent
        mostRecent = v;
    }

    return removed;

}

const vQueue vEdge::getSurf(int xl, int xh, int yl, int yh)
{
    xl = std::max(xl, 1);
    xh = std::min(xh, res.width-2);
    yl = std::max(yl, 1);
    yh = std::min(yh, res.height-2);

    vQueue subq;

    for(int y = yl; y <= yh; y++) {
        for(int x = xl; x <= xh; x++) {
            if(spatial[y][x])
                subq.push_back(spatial[y][x]);
        }
    }

    return subq;

}

vFuzzyEdge::vFuzzyEdge(int width, int height, double delta) :
    vEdge(width, height)
{
    this->delta = delta;
    scores.resize(height);
    for(int y = 0; y < height; y++) {
        scores[y].resize(width, 0);
    }


}
vQueue vFuzzyEdge::addEventToEdge(event<AddressEvent> v)
{

    if(!v) return vQueue();
    event<FlowEvent> vf = as_event<FlowEvent>(v);
    if(!vf) return vQueue();

    int x = vf->x; int y = vf->y;
    double vx = vf->vy; double vy = vf->vx;
    double vmag = sqrt(pow(vx, 2.0) + pow(vy, 2.0));
    //vx = vx / vmag; vy = vy / vmag;
    int px, py;

    //remove event at this location

    //double a = 0;
    double t = 0.708;//0.708;
    int f = 3;

    int y1 = -std::min(y, f);
    int y2 = std::min(res.height - 1 - y, f);
    int x1 = -std::min(x, f);
    int x2 = std::min(res.width - 1 - x, f);

    //double vx1 = vx + a * vy; double vy1 = vy - a * vx;
    //double vx2 = vx - a * vy; double vy2 = vy + a * vx;

    //remove events not on the perpendicular line
    for(int yi = y1; yi <= y2; yi++) {
        for(int xi = x1; xi <= x2; xi++) {
            double dcenter = sqrt(pow(yi, 2.0) + pow(xi, 2.0));
            if(dcenter > f) continue; //use LUT here

            //if(px < 0 || py < 0 || px > width-1 || py > height-1) continue;

            double d = (xi * vx + yi * vy) / vmag;
            double p = 0.4 * pow(2.718281, -pow(d, 2.0) / t) - 0.2;
            //p *= (f - dcenter/2) / f;
            px = x + xi; py = y + yi;
//            if(scores[py][px] < 0.5) {
//                scores[py][px] += p;
//                scores[py][px] = std::max(scores[py][px], 0.0);
//                if(scores[py][px] > 0.5) scores[py][px] = 1.0;
//            } else {
//                scores[py][px] += p;
//                scores[py][px] = std::min(scores[py][px], 1.0);
//                if(scores[py][px] < 0.5) scores[py][px] = 0.0;
//            }

            scores[py][px] += p;
            scores[py][px] = std::min(scores[py][px], 1.0);
            scores[py][px] = std::max(scores[py][px], 0.0);

//            double d1 = xi * vx1 + yi * vy1;
//            double d2 = xi * vx2 + yi * vy2;
//            if(fabs(d1) > t && fabs(d2) > t && d1 * d2 >= 0) {
//                scores[py][px] = std::max(scores[py][px] - 0.2, 0.0);
//            } else {
//                scores[py][px] = std::min(scores[py][px] + 0.2, 1.0);
//            }
        }
    }

    return vQueue();

}

const vQueue vFuzzyEdge::getSURF(int xl, int xh, int yl, int yh)
{
    xl = std::max(xl, 1);
    xh = std::min(xh, res.width-2);
    yl = std::max(yl, 1);
    yh = std::min(yh, res.height-2);

    vQueue subq;

    for(int y = yl; y <= yh; y++) {
        for(int x = xl; x <= xh; x++) {
            if(scores[y][x] > 0.5) {
                auto ae = make_event<AddressEvent>();
                ae->x = x; ae->y = y; ae->setChannel(0); ae->polarity = 0; ae->stamp = 0;
                subq.push_back(ae);
            }
        }
    }

    return subq;
}

/******************************************************************************/

void historicalSurface::initialise(int height, int width)
{
    surface.resize(width, height);
}

void historicalSurface::addEvent(event<> v)
{
    auto ae = is_event<AE>(v);
    bool error = false;

    if(ae->x > surface.width() - 1 || ae->y > surface.height() - 1) {
        yError() << "Pixel Out of Range: " << ae->getContent().toString();
        error = true;
    }

    int dt = ae->stamp - debugstamp;
    if(dt < 0 && dt > -vtsHelper::max_stamp*0.5 ) {
        if(q.size() > 1)
            yError() << "Stamp Out of Order: " << (*(q.end()-2))->stamp << " " << debugstamp << " " << ae->stamp;
        else
            yError() << "Stamp Out of Order: " << debugstamp << " " << ae->stamp;
        error = true;
    } else {
        debugstamp = ae->stamp;
    }

    if(!error) vTempWindow::addEvent(v);


}

vQueue historicalSurface::getSurface(int queryTime, int queryWindow)
{
    if(q.empty()) return vQueue();

    vQueue qret;
    int ctime = q.back()->stamp;
    int breaktime = queryTime + queryWindow;
    surface.zero();

    for(vQueue::reverse_iterator qi = q.rbegin(); qi != q.rend(); qi++) {
        auto v = is_event<AE>(*qi);
        if(surface(v->x, v->y)) continue;

        double cdeltat = ctime - v->stamp;
        if(cdeltat < 0) cdeltat += vtsHelper::max_stamp;
        if(cdeltat > breaktime) break;
        if(cdeltat > queryTime) {
            qret.push_back(*qi);
            surface(v->x, v->y) = 1;
        }
    }
    return qret;
}

vQueue historicalSurface::getSurface(int queryTime, int queryWindow, int d)
{
    if(q.empty()) return vQueue();

    auto v = is_event<AE>(q.back());
    return getSurface(queryTime, queryWindow, d, v->x, v->y);
}
vQueue historicalSurface::getSurface(int queryTime, int queryWindow, int d, int x, int y)
{
    if(q.empty()) return vQueue();
    return getSurface(queryTime, queryWindow, x - d, x + d, y - d, y + d);
}

vQueue historicalSurface::getSurface(int queryTime, int queryWindow, int xl, int xh, int yl, int yh)
{
    if(q.empty()) return vQueue();

    vQueue qret;
    int ctime = q.back()->stamp;
    int breaktime = queryTime + queryWindow;
    surface.zero();

    for(vQueue::reverse_iterator qi = q.rbegin(); qi != q.rend(); qi++) {
        auto v = is_event<AE>(*qi);
        if(surface(v->x, v->y)) continue;

        double cdeltat = ctime - v->stamp;
        if(cdeltat < 0) cdeltat += vtsHelper::max_stamp;
        if(cdeltat > breaktime) break;
        if(cdeltat > queryTime) {
            surface(v->x, v->y) = 1;
            if(v->x >= xl && v->x <= xh && v->y >= yl && v->y <= yh)
                qret.push_back(*qi);
        }
    }
    return qret;
}


}

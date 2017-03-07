#include <iCub/eventdriven/vWindow.h>

namespace ev {

vTempWindow::vTempWindow()
{
    tLower = vtsHelper::maxStamp() * 0.5;
    tUpper = vtsHelper::maxStamp() - tLower;
}

void vTempWindow::addEvent(event<> v)
{
    int ctime = v->getStamp();
    int upper = ctime + tUpper;
    int lower = ctime - tLower;

    while(q.size()) {
        int vtime = q.back()->getStamp();
        if((vtime >= ctime && vtime < upper) || vtime < lower) {
            q.pop_back();
        } else {
            break;
        }
    }

    while(q.size()) {

        int vtime = q.front()->getStamp();
        if((vtime >= ctime && vtime < upper) || vtime < lower) {
            q.pop_front();
        } else {
            break;
        }
    }

    q.push_back(v);
}

void vTempWindow::addEvents(const vQueue &events)
{
    vQueue::const_iterator qi;
    for(qi = events.begin(); qi != events.end() - 1; qi++)
        q.push_back(*qi);

    addEvent(events.back());
}

vQueue vTempWindow::getWindow()
{
    return q;
}

/******************************************************************************/

vROIWindow::vROIWindow()
{
    xl = 0;
    yh = 0;
    yl = 0;
    yh = 0;
    dt = 0;
}

void vROIWindow::addEvent(event<AddressEvent> v)
{
    int ctime = v->getStamp();
    int upper = ctime + dt;
    int lower = ctime - dt;

    while(q.size()) {

        int vtime = q.front()->getStamp();
        if((vtime >= ctime && vtime < upper) || vtime < lower) {
            q.pop_front();
        } else {
            break;
        }
    }

    if(v->getX() >= xl && v->getX() <= xh && v->getY() >= yl && v->getY() <= yh)
        q.push_back(v);
}

vQueue vROIWindow::getWindow(int xl, int xh, int yl, int yh, int dt)
{
    this->xl = xl;
    this->xh = xh;
    this->yl = yl;
    this->yh = yh;
    this->dt = dt;

    return q;
}

}

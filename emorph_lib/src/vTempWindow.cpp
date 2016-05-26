#include "iCub/emorph/vWindow.h"

namespace emorph {

vTempWindow::vTempWindow(int width, int height)
{
    this->width = width;
    this->height = height;
    tLower = vtsHelper::maxStamp() * 0.5;
    tUpper = vtsHelper::maxStamp() - tLower;
}

void vTempWindow::addEvent(vEvent &event)
{
    int ctime = event.getStamp();
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

    q.push_back(&event);
}

void vTempWindow::addEvents(const vQueue &events)
{
    vQueue::const_iterator qi;
    for(qi = events.begin(); qi != events.end() - 1; qi++)
        q.push_back(*qi);

    addEvent(*events.back());
}

vQueue vTempWindow::getWindow()
{
    return q;
}


}

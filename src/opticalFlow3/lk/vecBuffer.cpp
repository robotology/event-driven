#include "vecBuffer.h"

using namespace std;
using namespace yarp::os;
vecBuffer::vecBuffer()
{
	x=0;
    y=0;
	vx=0;
    vy=0;
}
vecBuffer::vecBuffer(int i_x, int i_y, double i_vx, double i_vy)
{
	x = i_x;
	y = i_y;
	vx = i_vx;
	vy = i_vy;
}
vecBuffer::~vecBuffer()
{
}
void vecBuffer::set_data(int i_x, int i_y, double i_vx, double i_vy)
{
	x = i_x;
	y = i_y;
	vx = i_vx;
	vy = i_vy;
}
bool vecBuffer::write(yarp::os::ConnectionWriter& connection)
{
	connection.appendInt(BOTTLE_TAG_LIST+BOTTLE_TAG_DOUBLE+BOTTLE_TAG_INT);
	connection.appendInt(4); // three elements
	connection.appendInt(x);
	connection.appendInt(y);
	connection.appendDouble(vx);
	connection.appendDouble(vy);
	connection.convertTextMode(); // if connection is text-mode, convert!
	return true;
}
bool vecBuffer::read(yarp::os::ConnectionReader& connection)
{
	connection.convertTextMode(); // if connection is text-mode, convert!
	int tag = connection.expectInt();
	if (tag!=BOTTLE_TAG_LIST+BOTTLE_TAG_DOUBLE+BOTTLE_TAG_INT)
		return false;
	int ct = connection.expectInt();
	if (ct!=4)
		return false;
	x = connection.expectInt();
    y = connection.expectInt();
    vx = connection.expectDouble();
    vy = connection.expectDouble();
	return true;
}


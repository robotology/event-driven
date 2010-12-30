#ifndef VECBUFFER
#define VECBUFFER

//#include <yarp/os/Portable.h>
#include <yarp/os/all.h>

#include <cstring>
//#include <cstring>

class vecBuffer:public yarp::os::Portable
{
public:
	vecBuffer();
	vecBuffer(int, int, double, double);
	~vecBuffer();
	virtual bool write(yarp::os::ConnectionWriter&);
	virtual bool read(yarp::os::ConnectionReader&);

	void set_data(int, int, double, double);

	inline int get_x(){return x;};
    inline int get_y(){return y;};
	inline double get_vx(){return vx;};
	inline double get_vy(){return vy;};
private:
	int x;
    int y;
	double vx;
    double vy;
};

#endif //VECBUFFER


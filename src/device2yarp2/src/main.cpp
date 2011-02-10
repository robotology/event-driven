#include "device2yarp.h"

int main(int argc, char* argv[])
{
	Network yarp;
	std::string in;
	bool end = false;
	bool _save = false;
	string deviceNum = "0";
	string fileName = "raw_events.bin";;
	switch(argc)
	{
	    case 4: fileName = argv[3];
	    case 3: _save = (bool)atoi(argv[2]);
	    case 2: deviceNum = argv[1];
	}
	C_device2yarp D2Y(deviceNum, _save, fileName);
	D2Y.start();
    while(!end)
    {
        std::cin >> in;
        if (in=="quit")
            end=true;
    }
	D2Y.stop();
    return 0;
}

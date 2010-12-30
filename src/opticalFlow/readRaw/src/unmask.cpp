#include "unmask.h"

using namespace std;
unmask::unmask()
{
	xmask = 0x000000fE;
	ymask = 0x00007f00;
	yshift = 8;
	xshift = 1;
	polshift = 0;
	polmask = 0x00000001;

	retinalSize = 128;

	wrapAdd = 0;

    argSYN        = new void*[12];
    sMapX         = 128;              argSYN[0]= &sMapX;
    sMapY         = 128;              argSYN[1]= &sMapY;
    neighLR       = 2;                argSYN[2]= &neighLR;
    stdDev        = 0.8;              argSYN[3]= &stdDev;
    threshold     = 2;                argSYN[4]= &threshold;
    alpha         = 5;                argSYN[5]= &threshold;
    tauC          = 5;                argSYN[6]= &tauC;
    tauD          = 250;              argSYN[7]= &tauD;
    accTime       = 5000;             argSYN[8]= &accTime;
                                      argSYN[9]= NULL;
//    void (*fn2call)(int, void**)= &objFlow.fnCall;  argSYN[9]= reinterpret_cast<void*>(fn2call);
    nApF      = 4;                    argSYN[10]= &nApF;
    selection = 99;/*0x00000063;*/    argSYN[11]= &selection;

    objSynapse    = synapse(12, argSYN);
    std::string portName="/image/opticalFlow:o";
    objSynapse.openPort(portName);
    args=new void*[4];

    firstT=-1;
    times.open("times.txt");
    clock_gettime(CLOCK_REALTIME, &start);
}

//unmask::unmask(void(*fn2call)(int, void**))
//{
//    ptrFn=fn2call;
//
//	xmask = 0x000000fE;
//	ymask = 0x00007f00;
//	yshift = 8;
//	xshift = 1;
//	polshift = 0;
//	polmask = 0x00000001;
//
//	retinalSize = 128;
//
//	wrapAdd = 0;
//}
unmask::~unmask()
{
    times << ssBuffer.str();
    times.close();

    objSynapse.closePort();
}
void unmask::unmaskData(char* i_buffer, int i_sz, int type)
{
//    cout << "unmask::unmaskData(...)" << endl;
    for (int j=0; j<i_sz; j+=type)
    {
        if( (type==4) && ((i_buffer[j+3]&0x80)==0x80) )
        { // timestamp bit 15 is one -> wrap
            // now we need to increment the wrapAdd

            wrapAdd+=0x4000/*L*/; //uses only 14 bit timestamps
            //System.out.println("received wrap event, index:" + eventCounter + " wrapAdd: "+ wrapAdd);
            //NumberOfWrapEvents++;
        }
        else if( (type==4) && ((i_buffer[j+3]&0x40)==0x40  ) )
        { // timestamp bit 14 is one -> wrapAdd reset
            // this firmware version uses reset events to reset timestamps
            //write(file_desc,reset,1);//this.resetTimestamps();
//            buffer_msg[0] = 6;
//            write(file_desc,buffer_msg,1);
            wrapAdd=0;
            // log.info("got reset event, timestamp " + (0xffff&((short)aeBuffer[i]&0xff | ((short)aeBuffer[i+1]&0xff)<<8)));
        }
        else
        {
//----------------------------Unmask the data----------------------------------------------------------//
//            cout << "\tExtraction of the data ..." << endl;
            if(type == 4)
            {
                part_1 = 0x000000FF&i_buffer[j];
                part_2 = 0x000000FF&i_buffer[j+1];

                blob = (part_1)|(part_2<<8);
                unmaskEvent(blob, cartX, cartY, polarity);

                part_3 = 0x000000FF&i_buffer[j+2];
                part_4 = 0x000000FF&i_buffer[j+3];

                timestamp = ((part_3)|(part_4<<8))/*&0x7fff*/;
                timestamp+=wrapAdd;
            }
            else if(type == 6)
            {
                part_1 = 0x000000FF&i_buffer[j];
                part_2 = 0x000000FF&i_buffer[j+1];

                blob = (part_2)|(part_1<<8);
                unmaskEvent(blob, cartX, cartY, polarity);

                part_3 = 0x000000FF&i_buffer[j+2];
                part_4 = 0x000000FF&i_buffer[j+3];
                part_5 = 0x000000FF&i_buffer[j+4];
                part_6 = 0x000000FF&i_buffer[j+5];

                timestamp = (part_6)|(part_5<<8)|(part_4<<16)|(part_3<<24);/*&0x7fff*/;
                timestamp+=wrapAdd;
            }
            else if(type==8)
            {
                part_1 = 0x000000FF&i_buffer[j];
                part_2 = 0x000000FF&i_buffer[j+1];
                part_3 = 0x000000FF&i_buffer[j+2];
                part_4 = 0x000000FF&i_buffer[j+3];

                blob = (part_4)|(part_3<<8)|(part_2<<16)|(part_1<<24);
//                blob = (0x000000FF&i_buffer[j+3])|((0x000000FF&i_buffer[j+2])<<8)|((0x000000FF&i_buffer[j+1])<<16)|((0x000000FF&i_buffer[j])<<24);

                unmaskEvent(blob, cartX, cartY, polarity);

                part_5 = 0x000000FF&i_buffer[j+4];
                part_6 = 0x000000FF&i_buffer[j+5];
                part_7 = 0x000000FF&i_buffer[j+6];
                part_8 = 0x000000FF&i_buffer[j+7];

                timestamp = (part_8)|(part_7<<8)|(part_6<<16)|(part_5<<24);/*&0x7fff*/;
//                timestamp = (0x000000FF&i_buffer[j+7])|((0x000000FF&i_buffer[j+6])<<8)|((0x000000FF&i_buffer[j+5])<<16)|((0x000000FF&i_buffer[j+4])<<24);/*&0x7fff*/;

                timestamp+=wrapAdd;
            }

//            cout    << "\tExtracted datas: "                << endl
//                    << "\t\tcartX: "        << cartX        << endl
//                    << "\t\tcartY: "        << cartY        << endl
//                    << "\t\tpolarity: "     << polarity     << endl
//                    << "\t\ttimestamp: "    << timestamp    << endl;
//            getchar();

            if(cartX>=5 && cartY>=5 && cartX<=123 && cartY<=123)
            {
//Calc computation time
                if(firstT==-1)
                {
                    oldT=timestamp;
                    firstT=timestamp;
                }
                clock_gettime(CLOCK_REALTIME, &current);
                if(timestamp-oldT>=5000)
                {
                    diff_time = ((current.tv_sec*1E9+current.tv_nsec)-(start.tv_sec*1E9+start.tv_nsec));
                    ssBuffer << timestamp << " " << timestamp-firstT << " " << diff_time << endl;
                    clock_gettime(CLOCK_REALTIME, &start);
                    oldT=timestamp;
                }
//                else
//                    ssBuffer << timestamp << " " << endl;
//END calc computation time
                args[0]=&cartX;
                args[1]=&cartY;
                args[2]=&polarity;
                args[3]=&timestamp;
//                ptrFn(4, plop);
//                cout << "call the synapse filter ..." << endl;
                objSynapse.fnCall(4, args);
            }
        }
    }
}
void unmask::unmaskEvent(unsigned int evPU, int& x, int& y, int& pol)
{
//	x = (short)(retinalSize-1) - (short)((evPU & xmask)>>xshift);
    x   = (evPU & xmask)>>xshift;
	y   = (evPU & ymask)>>yshift;
	pol = (((evPU & polmask)>>polshift)==0)?-1:1;	//+1 ON, -1 OFF
}

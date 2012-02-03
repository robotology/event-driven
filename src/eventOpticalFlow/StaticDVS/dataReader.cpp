// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author Fouzhan Hosseini
 * email:   fouzhan.hosseini@iit.it
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

#include <fstream>
#include <iostream>
#include <string>
#include <stdint.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/ResourceFinder.h>
#include <iCub/eventBuffer.h>
#include <iCub/eventConversion.h>

#define  SIZE_OF_DATA  30000

using namespace std;

using namespace yarp::os;

class EventReader : public RFModule{
    bool dvsCam;
    int bufferSize;
    string inFileName;
    ifstream inFile;
    yarp::os::BufferedPort<eventBuffer> outPort;
    char buffer [2][SIZE_OF_DATA];
    char * buffPtr;
    char datam [10];


    void convertToInt(char * in){
        for (int i = 0; i < 8; i++) {
            if ( in[i] >= '0' && in[i] <= '9' )
                in[i] = in[i] - '0';
            else{
                in[i] = in[i] - 'A';
                in[i] += 10;
            }
            //cout << (unsigned int) in[i]<< " ";
        }
        //cout << endl;
    }

    void loopICUBCam(){
        int buffer_idx;
        unsigned char oneByte;

        buffer_idx =0;

        while (!inFile.eof() && buffer_idx < bufferSize){
           //Read data from file and process the input data
           convertToInt(datam);
           for (int i = 6; i>= 0; i-=2) {
               oneByte = (datam[i] << 4) | datam [i+1];
               *(buffPtr + buffer_idx) = oneByte;
               buffer_idx++;
           }
           if (datam[0]==8 && datam[1] == 8){            
               inFile >> datam;
               continue;
           }

           inFile >> datam;

           convertToInt(datam);
           for (int i = 6; i >= 0; i-=2) {
               oneByte = (datam[i] << 4) | datam [i+1];
               *(buffPtr + buffer_idx) = oneByte;
               buffer_idx++;
           }

           inFile >> datam;
        }
       //write data to the port
       eventBuffer & outObj = outPort.prepare();
       //outObj = eventBuffer(buffPtr, buffer_idx);
       outObj.set_data(buffPtr, buffer_idx);
       outPort.write();

//       yarp::os::Time::delay(.001);


       if (buffPtr == buffer[0])
           buffPtr = buffer[1];
       else
           buffPtr = buffer[0];

     /*unmask unmasker;
       uint32_t* buf2 = (uint32_t*)buffPtr;
       short cartX, cartY, polarity, camera;

       for (int evt = 0; evt < 4; ++evt) {
          unsigned long blob = buf2[2 * evt];
          unsigned long timestamp = buf2[2 * evt + 1];
          printf("  %d ", (unsigned char)*(buffPtr + evt*8));
          printf(" %d ", (unsigned char)*(buffPtr + evt*8 + 1));
          printf(" %d ", (unsigned char)*(buffPtr + evt*8 + 2));
          printf(" %d \n", (unsigned char) *(buffPtr + evt*8 + 3));
          printf("  %d ", (unsigned char)*(buffPtr + evt*8 + 4));
          printf(" %d ", (unsigned char)*(buffPtr + evt*8 + 5));
          printf(" %d ", (unsigned char)*(buffPtr + evt*8 + 6));
          printf(" %d \n", (unsigned char) *(buffPtr + evt*8 + 7));
          printf("  %08x ",  blob);
          printf(" %08x \n ", timestamp);
          blob &= 0xFFFF;

          unmasker.unmaskEvent((unsigned int) blob, cartX, cartY, polarity, camera);
          cout << cartX << " " << cartY << " " << polarity << " " << camera << endl;
       }*/



    }

    void loopSingleCam(){

        int buffer_idx;
        char oneEvent [10];
        uint32_t a, t;

        buffer_idx =0;

        cout << "loop  "  << inFile.tellg()<< endl ;

        while (!inFile.eof() && buffer_idx < bufferSize){ // 8192 bytes -> 1024 events
           //Read data corresponding to one event form the input file
            inFile.read(oneEvent, 8);
            //addr
            for (int i = 3; i >= 0; --i) {
                  *(buffPtr + buffer_idx++) = oneEvent[i];
            }
            //timestamp
            for (int i = 7; i >= 4; --i) {
              *(buffPtr + buffer_idx++) = oneEvent[i];
            }

        }
       //write data to the port
       eventBuffer & outObj = outPort.prepare();
       outObj.set_data(buffPtr, buffer_idx);
       outPort.write();

       if (buffPtr == buffer[0])
           buffPtr = buffer[1];
       else
           buffPtr = buffer[0];


    }


     void readHeaderFile(){
         char line [100];
         int pos;

         inFile.getline(line, 100);

         char tok[10];
         strcpy(tok, "#!AER-DAT");
         float version=0;
         while (line[0] == '#'){

             if (strncmp(line,tok, 5)==0)
             {
                 printf("line==tok\n");
                 sscanf(line,"%*9s%f", &version);
             }
             printf("line : %s | tok : %s\n",line, tok); // print line using \n for newline, discarding CRLF written by java under windows

             pos = inFile.tellg();
             cout << pos << endl;
             inFile.getline(line, 100);

         }

         inFile.close();
         inFile.open(inFileName.c_str(),  ifstream::binary);
         inFile.seekg(pos, ios::beg);
         cout << inFile.tellg() << endl;
     }


public:
    bool configure(ResourceFinder & rf){

        //inFileName = "/home/fuozhan/Desktop/work/RecordedData/PC104/test1_v3.txt";
       // inFileName = "/home/fuozhan/Desktop/iCubReal/dump_hand.txt";
       // inFileName = "/home/fuozhan/Desktop/iCubReal/firstTr.txt";
       // inFileName = "/home/fuozhan/Desktop/iCubReal/secondTr.txt";
        inFileName = "/home/fuozhan/Desktop/iCubReal/dumpCar1.txt";
        
        dvsCam = true; // true to work with the robot -- false to work with the stand alone camera
        bufferSize = 2048; // 16384 -> 2048 events, 8192 bytes -> 1024 events , 4096-> 512 events

        if (dvsCam){
            inFile.open(inFileName.c_str(), ifstream::in);
            inFile >> datam;
        }
        else{
            inFile.open(inFileName.c_str(), ifstream::binary);
            readHeaderFile();
        }

        outPort.open("/eventReader/artificialRetina0:o");
        buffPtr = buffer[0];

        //loopSingleCam();
        return true;
    }

    bool interruptModule(){
        outPort.interrupt();
        if (inFile.is_open())
            inFile.close();
        cout << "Fake DVS camera module: interrupt function is called." << endl;
        return true;
    }

    bool updateModule(){
        if (!inFile.eof()){
            if(dvsCam)
                loopICUBCam();
            else{
                loopSingleCam();
            }
        }
        else
           cout << "end of file " << endl;

        return true;
    }

    double getPeriod()
    {
       /* module periodicity (seconds), called implicitly by myModule */
       return .005;
    }


    bool close(){
        //outPort.setStrict(false);
        //outPort.write();
        outPort.close();
        if (inFile.is_open())
            inFile.close();

        cout << "Fake DVS camera module: close function is called." << endl;
        return true; 
    }

    ~EventReader(){
       cout << "Fake DVS camera module: destructor is called." << endl;
    }

};

/*
class MyThread : public yarp::os::RateThread {
public:
    MyThread():RateThread(1) { }
    ~MyThread()
    {
        cout<<"Dead"<<endl;
    }

    virtual bool threadInit()
    {
        outPort.open("/eventReader/artificialRetina0:o");
        return true;
    }


    virtual void run()
    {

        buffPtr = buffer[0];
        eventBuffer & outObj = outPort.prepare();
        outObj.set_data(buffPtr, 30000);
        outPort.write();
        cout<<"outPort.write();"<<endl;
    }

    void kill()
    {
        outPort.close();
    }

private:
    bool dvsCam;
    int bufferSize;
    string inFileName;
    ifstream inFile;
    yarp::os::BufferedPort<eventBuffer> outPort;
    char buffer [2][SIZE_OF_DATA];
    char * buffPtr;
    char datam [10];

};
*/

int main(){
    Network yarp;

    EventReader * eReader = new EventReader();
    ResourceFinder rf;

   if (!eReader->configure(rf))
   {
       fprintf(stderr, "Error configuring Event Reader module returning\n");
       return -1;
   }

   eReader->runModule();

   delete eReader;

   /* MyThread my;
    my.start();
    cout<<"press enter"<<endl;
    int x;
    cin>>x;
    cout<<"killing"<<endl;
    my.kill();
    my.stop();*/

   cout << " last line in main" << endl;
   return 0;
}




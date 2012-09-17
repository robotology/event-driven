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
#include <iCub/emorph/eventBuffer.h>
#include <iCub/emorph/eventConversion.h>

#define  SIZE_OF_DATA  30000

using namespace std;

using namespace yarp::os;
using namespace emorph::ebuffer;

class EventReader : public RFModule{
    bool dvsCam;
    int jearFileType;
    int bufferSize;
    string inFileName;
    ifstream inFile;
    //yarp::os::BufferedPort<eventBuffer> outPort;
    yarp::os::Port outPort;
    char buffer [2][SIZE_OF_DATA];
    char * buffPtr;
    char datam [10];
    unsigned int prvTime;
    unsigned int cntTime;


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
        if (outPort.getOutputCount()){
            eventBuffer outObj = eventBuffer(buffPtr, buffer_idx);
            outPort.write(outObj);
        }


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

    void loopSingleCam(int fileTye){

        int buffer_idx;
        char oneEvent [10];
        uint32_t a, t;

        buffer_idx =0;
        bool writeSuccess = false;

     //   cout << "loop  "  << inFile.tellg()<< endl ;

        while (!inFile.eof() && buffer_idx < bufferSize){ // 8192 bytes -> 1024 events
//        	cout << inFile.tellg() <<  " " << buffer_idx << endl;
           //Read data corresponding to one event form the input file


//            printf("  %d ", (unsigned char)*(oneEvent ));
//			printf(" %d ", (unsigned char)*(oneEvent +  1));
//			printf(" %d ", (unsigned char)*(oneEvent+  2));
//			printf(" %d ", (unsigned char) *(oneEvent + 3));
//			printf("  %d ", (unsigned char)*(oneEvent  + 4));
//			printf(" %d ", (unsigned char)*(oneEvent + 5));
//			printf(" %d ", (unsigned char)*(oneEvent + 6));
//			printf(" %d \n", (unsigned char) *(oneEvent  + 7));


            //addr
            switch (fileTye) {
				case 1:
					//2 bytes for address
					inFile.read(oneEvent, 6);
					for (int i = 1; i >= 0; --i) {
						*(buffPtr + buffer_idx++) = oneEvent[i];
					}
					*(buffPtr + buffer_idx++) = 0;
					*(buffPtr + buffer_idx++) = 0;
//			  cout << "1---- " << buffer_idx<< endl;
					//timestamp
					for (int i = 5; i >= 2; --i) {
					  *(buffPtr + buffer_idx++) = oneEvent[i];
					}
//            cout << "2 ---- " << buffer_idx<< endl;
					break;
				case 2:
					inFile.read(oneEvent, 8);
					//4 bytes for timestamp, 4 bytes for Address
					for (int i = 3; i >= 0; --i) {
						*(buffPtr + buffer_idx++) = oneEvent[i];
					}
   				    //timestamp
					for (int i = 7; i >= 4; --i) {
					  *(buffPtr + buffer_idx++) = oneEvent[i];
					}
					break;
			}
        }
       //write data to the port
        if (outPort.getOutputCount()){
        	eventBuffer outObj = eventBuffer(buffPtr, buffer_idx);
            writeSuccess = outPort.write(outObj);
            cout << writeSuccess << endl;
        }

//        unmask unmasker;
//		uint32_t* buf2 = (uint32_t*)buffPtr;
//		short cartX, cartY, polarity, camera;
//
//		 for (int evt = 0; evt < bufferSize/8; ++evt) {
//			unsigned long blob = buf2[2 * evt];
//			unsigned long timestamp = buf2[2 * evt + 1];
//			printf("  %08x ",  blob);
//			printf(" %08x \n ", timestamp);
//
//			blob &= 0xFFFF;
//			unmasker.unmaskEvent((unsigned int) blob, cartX, cartY, polarity, camera);
//			cout << cartX << " " << cartY << " " << polarity << " " << camera << endl;
//        }

       if (buffPtr == buffer[0])
           buffPtr = buffer[1];
       else
           buffPtr = buffer[0];

    }

    void loopSingleCamTime(int fileTye){

            int buffer_idx;
            char oneEvent [10];
            uint32_t a, t;


            buffer_idx =0;
            bool writeSuccess = false;

            while (!inFile.eof() && cntTime - prvTime < 1000){ // 8192 bytes -> 1024 events
                //Read data corresponding to one event form the input file
                //addr
                switch (fileTye) {
                    case 1:
                        //2 bytes for address
                        inFile.read(oneEvent, 6);
                        for (int i = 1; i >= 0; --i) {
                            *(buffPtr + buffer_idx++) = oneEvent[i];
                        }
                        *(buffPtr + buffer_idx++) = 0;
                        *(buffPtr + buffer_idx++) = 0;
                        //timestamp
                        for (int i = 5; i >= 2; --i) {
                          *(buffPtr + buffer_idx++) = oneEvent[i];
                        }
                        cntTime =  *((uint32_t*) (buffPtr + buffer_idx - 4));
                        break;
                    case 2:
                        inFile.read(oneEvent, 8);
                        //4 bytes for timestamp, 4 bytes for Address
                        for (int i = 3; i >= 0; --i) {
                            *(buffPtr + buffer_idx++) = oneEvent[i];
                        }
                        //timestamp
                        for (int i = 7; i >= 4; --i) {
                          *(buffPtr + buffer_idx++) = oneEvent[i];
                        }
                        cntTime =  *((uint32_t*) (buffPtr + buffer_idx - 4));
                        break;
                }
            }

           //write data to the port
            if (outPort.getOutputCount()){
                eventBuffer outObj = eventBuffer(buffPtr, buffer_idx);
                writeSuccess = outPort.write(outObj);
                cout << writeSuccess << endl;
            }

            prvTime = cntTime;

           if (buffPtr == buffer[0])
               buffPtr = buffer[1];
           else
               buffPtr = buffer[0];

        }



     void readHeaderFile(int fileTye){
         char line [100];
         int pos, buffer_idx = 0;

         cout << "reading header file ..." << endl;

         inFile.getline(line, 100);

         char tok[10];
         strcpy(tok, "#!AER-DAT");
         float version=0;
         pos = 0;
         while (line[0] == '#'){

             if (strncmp(line,tok, 5)==0)
             {
                 printf("line==tok\n");
                 sscanf(line,"%*9s%f", &version);
             }
             printf("line : %s | tok : %s\n",line, tok); // print line using \n for newline, discarding CRLF written by java under windows

             pos = inFile.tellg();
             inFile.getline(line, 100);

         }

         switch (fileTye) {
             case 1:
                 //2 bytes for address
                 //timestamp
                 for (int i = 5; i >= 2; --i) {
                   *(buffPtr + buffer_idx++) = line[i];
                 }
                 prvTime =  *((uint32_t*) buffPtr);
                 break;
             case 2:
                 //4 bytes for timestamp, 4 bytes for Address
                 //timestamp
                 for (int i = 7; i >= 4; --i) {
                   *(buffPtr + buffer_idx++) = line[i];
                 }
                 prvTime =  *((uint32_t*) buffPtr);
                 break;
         }


         inFile.close();
         inFile.open(inFileName.c_str(),  ifstream::binary);
         inFile.seekg(pos, ios::beg);
         cout << inFile.tellg() << endl;
     }


public:
    bool configure(ResourceFinder & rf){

        string moduleName;
        moduleName =  rf.check("name", Value("artRetina"), "module name (String)").asString();

        inFileName = "";
        inFileName = rf.check("inFile",
                                    Value("/tmp/dump.txt"),
                                    "Input  input File Name (string)").asString();
        cout << "FileName : " <<inFileName << endl;

        jearFileType = rf.check("fileType",
                                  Value(0),
                                  "Input  input File type (int)").asInt();
        // true to work with the robot -- false to work with the stand alone camera
        if (jearFileType == 0)
        	dvsCam = true;
        else
        	dvsCam = false;

        buffPtr = buffer[0];

        bufferSize = 512;	; // 16384 -> 2048 events, 8192 bytes -> 1024 events , 4096-> 512 events

        if (dvsCam){
            inFile.open(inFileName.c_str(), ifstream::in);
            if ( inFile.fail() ){  //The file is not opened correctly
                cerr << "Sorry! cannot open the input file. "<< endl;
                return false;
            }

            inFile >> datam;
        }
        else{
        	inFile.open(inFileName.c_str(), ifstream::binary);
        	if ( inFile.fail() ){ //The file is not opened correctly
        	    cerr << "Sorry! cannot open the input file. "<< endl;
        	    return false;
        	}

//        	if (jearFileType == 2)
               readHeaderFile(jearFileType);

        }


        outPort.open("/eventReader/artificialRetina0:o");

        outPort.setTimeout(5.0);

//        for (int j = 0; j < 1000; ++j) {
//              loopSingleCam(jearFileType);
//       }

        cntTime = prvTime = 0;

        return true;
    }



    bool updateModule(){
        if (!inFile.eof()){
            if(dvsCam){
                loopICUBCam();
            }
            else{
            	loopSingleCamTime(jearFileType);
            }
        }
        else
           cout << "end of file " << endl;

        return true;
    }

    double getPeriod()
    {
       /* module periodicity (seconds), called implicitly by myModule */
       return .001;
    }

    bool interruptModule(){
        outPort.interrupt();
        if (inFile.is_open())
            inFile.close();
        cout << "Fake DVS camera module: interrupt function is called." << endl;
        return true;
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


int main(int argc, char * argv []){
    Network yarp;

    EventReader eReader;
    ResourceFinder rf;

    rf.setVerbose(true);
    rf.configure("ICUB_ROOT", argc, argv);
    if (!eReader.configure(rf))
    {
        fprintf(stderr, "Error configuring Event Reader module returning\n");
        return -1;
    }

   eReader.runModule();

   cout << " last line in main" << endl;
   return 0;
}




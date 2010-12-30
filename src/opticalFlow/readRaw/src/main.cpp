#include "readRaw.h"

//#define _DEBUG
using namespace yarp::os;
using namespace std;
int main(int argc, char* argv[])
{
    Network yarp;
    if(argc==2)
    {
        unmask objUnmask;
        FILE* data = NULL;
        char datFile[150];
        char line[100];
        strcpy(datFile, argv[1]);
        data = fopen(datFile, "rb");
        int bof=ftell(data);
        fgets(line, 100, data);
        char tok[10];
        strcpy(tok, "#!AER-DAT");
        float version=0;
        while(line[0]=='#')
        {
            if (strncmp(line,tok, 5)==0)
            {
                printf("line==tok\n");
                sscanf(line,"%*9s%f", &version);
            }
            printf("line : %s | tok : %s\n",line, tok); // print line using \n for newline, discarding CRLF written by java under windows
            bof=ftell(data);
            fgets(line, 100, data);   //gets the line including line ending chars
        }
//        printf("version : %f\n, %d", version, (int)version);
//        printf("bof = %d\n", bof);
        fseek(data, 0, SEEK_END);
        int size=ftell(data);

        fseek(data, bof, SEEK_SET);//rewind(data);
        char* buffer = new char[size-bof]; //= (char*) malloc (sizeof(char)*size);
        int nbPEvts = (size-bof)/8;
        printf("nb evts = %d\n", nbPEvts);
//        int res = fread(buffer, 8, nbPEvts, data);
        int res = fread(buffer, 1, size-bof, data);
        printf("Number of byte successfully read : %d\n", res);
        int type=6;
        switch((int)version)
        {
            case 0:     printf("No #!AER-DAT version header found, assuming 16 bit addresses\n");
                        type=6;
                        break;
            case 1:     printf("Addresses are 16 bit\n");
                        type=6;
                        break;
            case 2:     printf("Addresses are 32 bit\n");
                        type=8;
                        break;
            default:    printf("Unknown file version %f\n",version);
                        type=6;
                        break;
        }
//        while(1)
//            objUnmask.unmaskData(buffer, nbPEvts, type);
            objUnmask.unmaskData(buffer, size-bof, type);
    }
    else
    {
        std::string in;
    #ifdef _DEBUG
        cout << "Create a C_yarpViewer instance" << endl;
    #endif
        readRaw objRR;
    #ifdef _DEBUG
        cout << "Register the instance to the callback" << endl;
    #endif
        objRR.useCallback();
    #ifdef _DEBUG
        cout << "Open the port for listening" << endl;
    #endif
        objRR.open("/image/opticalFlow:i");
    #ifdef _DEBUG
        cout << "Connect the 'read' and 'write port'" << endl;
    #endif
    //	Network::connect("/icub/retina0:o","/image/opticalFlow:i");
        bool end = false;
        //cvNamedWindow("Events", 0);
        while(!end)
        {
            std::cin >> in;
            if (in=="quit")
                end=true;
        }
    #ifdef _DEBUG
        cout << "Close the listening port" << endl;
    #endif
        objRR.close();
    #ifdef _DEBUG
        cout << "Unregister form the callback" << endl;
    #endif
        objRR.disableCallback();
    }
    return 0;
}

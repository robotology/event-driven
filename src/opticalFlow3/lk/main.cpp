#include <yarp/os/Network.h>

#include <iostream>

#include "unmask.h"
#include "readRaw.h"

int main(int argc, char* argv[])
{
    yarp::os::Network yarp;
    
    int channel=-1;

    if (argc==2)
    {
        std::string param(argv[1]);
        if (param=="--R")
        {
            channel=1;
            argc=0;
        }
        else if (param=="--L")
        {
            channel=0;
            argc=0;
        }
    }

    if(argc==2)
    {
        FILE* data = fopen(argv[1],"rb");
        int bof=ftell(data);

        char line[100];
        fgets(line,100,data);

        float version=0.0f;

        while(line[0]=='#')
        {
            if (strncmp(line,"#!AER-DAT",5)==0)
            {
                sscanf(line,"%*9s%f",&version);
            }

            printf("%s\n",line);
            bof=ftell(data);
            fgets(line,100,data);
        }

        printf("version = %f\n",version);

        fseek(data,0,SEEK_END);
        int size=ftell(data);

        fseek(data,bof,SEEK_SET);
        
        unsigned char* buffer=new unsigned char[size-bof];
        
        int nbPEvts=(size-bof)/8;

        printf("nb evts = %d\n", nbPEvts);

        int res=fread(buffer,1,size-bof,data);
        
        printf("Number of byte successfully read : %d\n",res);
        
        int type=8;
        switch((int)version)
        {
            case 0:     printf("No #!AER-DAT version header found, assuming 32 bit addresses\n");
                        type=8;
                        break;

            case 1:     printf("Addresses are 16 bit\n");
                        type=6;
                        break;

            case 2:     printf("Addresses are 32 bit\n");
                        type=8;
                        break;

            default:    printf("Unknown file version %f\n",version);
                        type=8;
                        break;
        }

        Unmask unmask(channel);
        unmask.unmaskData((unsigned int*)buffer,size-bof);
    }
    else
    {
        readRaw objRR(channel);
        objRR.useCallback();
        objRR.open("/optflow/events:i");
        printf("Running\n");
        fflush(stdout);

        std::string in;
        while(true)
        {
            std::cin >> in;

            if (in=="quit") break;
        }

        std::cout << "Close the listening port" << std::endl;
        objRR.close();
    
        std::cout << "Unregister form the callback" << std::endl;
        objRR.disableCallback();
    }

    return 0;
}

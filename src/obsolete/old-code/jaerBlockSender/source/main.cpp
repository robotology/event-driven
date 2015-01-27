#include "jaerBlockSender.hpp"

#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Vocab.h>

#define VOCAB_SEND VOCAB4('s','e','n','d')
#define VOCAB_QUIT VOCAB4('q','u','i','t')
#define VOCAB_ONCE VOCAB4('o','n','c','e')
#define VOCAB_FULL VOCAB4('f','u','l','l')

#define MAXCMD 500

using namespace std;
using namespace yarp::os;
int main(int argc, char *argv[])
{
    Network yarpNet;
    jaerBlockSender jBlockSender;

    yarp::os::Property params;
    params.fromCommand(argc, argv);
    if(!params.check("send") || !params.check("file"))
    {
        fprintf(stderr, "Error param\n");
        return -1;
    }
    string file=params.find("file").asString().c_str();
    int wait=0;
    if(params.check("wait"))
        wait=params.find("wait").asInt();

    switch(params.find("send").asVocab())
    {
        case VOCAB_ONCE:{
                            cerr << "\t\tRecognize \"ONCE\" parameter" << endl;
                            int res=jBlockSender.load(file.c_str());
                            if(res!=-1)
                            {
                                if(!params.check("size"))
                                    return -1;
                                if(wait)
                                {
                                    cout << "Press 'enter' key to send the seq" << endl;
                                    char c='\0';
                                    while(c!='\n')
                                        c=getchar();
                                }
                                jBlockSender.send(params.find("size").asInt());
                            }
                        }
                        break;
        case VOCAB_FULL:{
                            cerr << "\t\tRecognize \"FULL\" parameter" << endl;
                            int res=jBlockSender.load(file.c_str());
                            if(res!=-1)
                            {
                                if(!params.check("lgth"))
                                    return -1;
                                jBlockSender.prepareData((uint)params.find("lgth").asInt());
                                if(wait)
                                {
                                    cout << "Press 'enter' key to send the seq" << endl;
                                    char c='\0';
                                    while(c!='\n')
                                        c=getchar();
                                }
                                jBlockSender.sendBlocks(0, -1, (uint)params.find("lgth").asInt());
                            }
                        }
                        break;
        default:{
                    cerr << endl << "\tsend command expect once or full arg";
                }
                break;
    }
}

// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Rea Francesco
 * email:   francesco.rea@iit.it
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

#if !defined(GCAMVIEW_H)
#define GCAMVIEW_H

//=============================================================================
// YARP Includes - Class Specific
//=============================================================================
#include <yarp/sig/Image.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
//=============================================================================
// GTK Includes 
//=============================================================================
#include <gtk/gtk.h>
#include "viewerResources.h"
#include <string>

#define COMMAND_VOCAB_SYTH   VOCAB4('s','y','t','h')
#define COMMAND_VOCAB_SYTA   VOCAB4('s','y','t','a')
#define COMMAND_VOCAB_SYPA   VOCAB4('s','y','p','a')
#define COMMAND_VOCAB_SYPH   VOCAB4('s','y','p','h')
#define COMMAND_VOCAB_TPB    VOCAB3('t','p','b')
#define COMMAND_VOCAB_CDR    VOCAB3('c','d','r')
#define COMMAND_VOCAB_CDS    VOCAB3('c','d','s')
#define COMMAND_VOCAB_CDP    VOCAB3('c','d','p')
#define COMMAND_VOCAB_RPX    VOCAB3('r','p','x')
#define COMMAND_VOCAB_RPY    VOCAB3('r','p','y') 
#define COMMAND_VOCAB_IFR    VOCAB3('i','f','r')
#define COMMAND_VOCAB_IFT    VOCAB3('i','f','t')
#define COMMAND_VOCAB_IFL    VOCAB3('i','f','l'    )
#define COMMAND_VOCAB_CDOF   VOCAB4('c','d','o','f')
#define COMMAND_VOCAB_SYPW   VOCAB4('s','y','p','w')
#define COMMAND_VOCAB_SYW    VOCAB3('s','y','w')
#define COMMAND_VOCAB_CDON   VOCAB4('c','d','o','n')
#define COMMAND_VOCAB_CDD    VOCAB3('c','d','d')
#define COMMAND_VOCAB_EMCH   VOCAB4('e','m','c','h')
#define COMMAND_VOCAB_EMCT   VOCAB4('e','m','c','t')
#define COMMAND_VOCAB_CDI    VOCAB3('c','d','i')
#define COMMAND_VOCAB_CDRG   VOCAB4('c','d','r','g')
#define COMMAND_VOCAB_SELF   VOCAB4('s','e','l','f')
#define COMMAND_VOCAB_FOLL   VOCAB4('f','o','l','l')
#define COMMAND_VOCAB_ARBP   VOCAB4('a','r','b','p')
#define COMMAND_VOCAB_EMVL   VOCAB4('e','m','v','l')
#define COMMAND_VOCAB_CDC    VOCAB3('c','d','c')
#define COMMAND_VOCAB_EMVH   VOCAB4('e','m','v','h')

#define COMMAND_VOCAB_SAVE   VOCAB4('s','a','v','e')
#define COMMAND_VOCAB_LOAD   VOCAB4('l','o','a','d')
#define COMMAND_VOCAB_BIAS   VOCAB4('b','i','a','s')
#define COMMAND_VOCAB_PROG   VOCAB4('p','r','o','g')



//-------------------------------------------------
// Callbacks
//-------------------------------------------------
gboolean forceDraw(gpointer data);
// Timeout CB
gint timeout_CB (gpointer data);
// Main Window close CB
gboolean delete_event( GtkWidget *widget, GdkEvent *event, gpointer data );
//  Window redraw CB
gint expose_CB (GtkWidget *widget, GdkEventExpose *event, gpointer data);
// Click on Drawinf Area CB
gint clickDA_CB (GtkWidget *widget, GdkEventButton *event, gpointer data);
// Menubar CBs
gint menuFileQuit_CB(GtkWidget *widget, gpointer data);
gint menuHelpAbout_CB(GtkWidget *widget, gpointer data);
gint menuImageInterval_CB(GtkWidget *widget, gpointer data);
gint menuFileSingle_CB(GtkWidget *widget, GdkEventExpose *event, gpointer data);

void setTimedMode(guint dT);

void setSynchroMode();

void saveImageNow();

//-------------------------------------------------
// Non Modal Dialogs
//-------------------------------------------------
GtkWidget* createSaveSingleDialog();
GtkWidget* createSaveSetDialog();
//-------------------------------------------------
// Main Window Menubar
//-------------------------------------------------
GtkWidget* createMenubar();
//-------------------------------------------------
// Main Window Statusbar
//-------------------------------------------------
void updateStatusbar(GtkWidget *statusbar, gchar *msg);
//-------------------------------------------------
// Main Window
//-------------------------------------------------
GtkWidget* createMainWindow(void);
//-------------------------------------------------
// Service Functions
//-------------------------------------------------
bool getImage();
void parseOptFile(char *fileName);
bool parseParameters(int argc, char *argv[]);
void saveOptFile(char *fileName);
void setOptionsToDefault();
bool openPorts();
void closePorts();
bool setUp();
void cleanExit();
void printHelp();

//-------------------------------------------------
// Global Variables
//-------------------------------------------------
// main window 
GtkObject *adjSYTH,  *adjSYTA,  *adjSYPA,  *adjSYPH,  *adjTPB;
GtkObject *adjCDR,  *adjCDS,  *adjCDP,  *adjRPX;
GtkObject *adjRPY, *adjIFR, *adjIFT, *adjIFL, *adjCDOF, *adjSYPW, *adjSYW;
GtkObject *adjCDON, *adjCDD, *adjEMCH, *adjEMCT, *adjCDI, *adjCDRG, *adjSELF;
GtkObject *adjFOLL, *adjARBP, *adjEMVL, *adjCDC, *adjEMVH;
GtkObject *adjMotion;
GtkWidget *entrySYTH, *entrySYTA, *entrySYPA, *entrySYPH, *entryTPB ; 
GtkWidget *entryCDR , *entryCDS , *entryCDP , *entryRPX , *entryRPY ; 
GtkWidget *entryIFR , *entryIFT , *entryIFL , *entryCDOF, *entrySYPW; 
GtkWidget *entrySYW , *entryCDON, *entryCDD , *entryEMCH, *entryEMCT; 
GtkWidget *entryCDI , *entryCDRG, *entrySELF, *entryFOLL, *entryARBP; 
GtkWidget *entryEMVL, *entryCDC , *entryEMVH;
GtkWidget *entrySAVELOAD, *entryDUMP;

FILE* fout;                                                                      // file where the biases are saved
std::string filename;                                                            // name of the file where biases are saved
const gchar *entry_file;                                                         // name of the file where biased are saved
const gchar *entry_dump;                                                         // name of the file where the events will be dumped
yarp::os::Semaphore mutex;

int c=0;

//-------------------------------------------------
// Program Options 
//-------------------------------------------------
struct mOptions
{
    unsigned int    refreshTime;
    std::string     portName;
    char            networkName[256];
    int             windWidth;
    int             windHeight;
    int             posX;
    int             posY;
    char            fileName[256];
    int             saveOnExit;
    std::string           outPortName;
    char            outNetworkName[256];
    int             outputEnabled;
    bool            synch;
};
typedef struct mOptions pgmOptions;



#endif // #if !defined(GCAMVIEW_H)

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


#include "asvBiasInterface.h"

#include <yarp/os/Property.h> 
#include <yarp/os/Network.h> 
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>

#include <string.h>
#include <sstream>
#include <cstdlib>


#define COMMAND_VOCAB_ON    VOCAB2('o','n')
#define COMMAND_VOCAB_OFF   VOCAB3('o','f','f')
#define COMMAND_VOCAB_SET   VOCAB3('s','e','t')
#define COMMAND_VOCAB_GET   VOCAB3('g','e','t')
#define COMMAND_VOCAB_RIGHT VOCAB4('r','i','g','h')
#define COMMAND_VOCAB_LEFT  VOCAB4('l','e','f','t')
#define COMMAND_VOCAB_DUMP  VOCAB4('d','u','m','p')


#define SYTH_DEFAULT_VALUE    8053457
#define SYTA_DEFAULT_VALUE    8053457
#define SYPA_DEFAULT_VALUE    8053457
#define SYPH_DEFAULT_VALUE    8053457
#define TPB_DEFAULT_VALUE     16777215
#define CDR_DEFAULT_VALUE     944
#define CDS_DEFAULT_VALUE     20
#define CDP_DEFAULT_VALUE     5
#define RPX_DEFAULT_VALUE     8053457
#define RPY_DEFAULT_VALUE     16777215
#define IFR_DEFAULT_VALUE     639172
#define IFT_DEFAULT_VALUE     30108
#define IFL_DEFAULT_VALUE     20
#define CDOF_DEFAULT_VALUE    133
#define SYPW_DEFAULT_VALUE    0
#define SYW_DEFAULT_VALUE     0
#define CDON_DEFAULT_VALUE    639172
#define CDD_DEFAULT_VALUE     30108
#define EMCH_DEFAULT_VALUE    30108
#define EMCT_DEFAULT_VALUE    30108
#define CDI_DEFAULT_VALUE     160712
#define CDRG_DEFAULT_VALUE    101508
#define SELF_DEFAULT_VALUE    500
#define FOLL_DEFAULT_VALUE    52458
#define ARBP_DEFAULT_VALUE    16777215
#define EMVL_DEFAULT_VALUE    160712
#define CDC_DEFAULT_VALUE     52458
#define EMVH_DEFAULT_VALUE    52458


using namespace yarp::os;
using namespace std;

GtkWidget *mainWindow=0;
static GdkPixbuf *frame = NULL;

BufferedPort<yarp::sig::FlexImage> *ptr_inputPort=0;


std::string* command;   //reference to the string refering to the last command to send

int _frameN;            //Frame Number
bool _savingSet;        //Save Set of Images mode
// 
//yarp::os::BufferedPort<yarp::os::Bottle> *_pOutPort=0; //Output Point Port
Port* _pOutPort=0;
yarp::os::Bottle _outBottle;//Output Bottle Container
pgmOptions _options; //option set by the config file

GtkWidget *saveSingleDialog;
GtkWidget *saveSetDialog;
GtkWidget *menubar;
GtkWidget *fileMenu, *imageMenu, *helpMenu;
GtkWidget *fileItem, *helpItem;
GtkWidget *fileSingleItem, *fileSetItem, *fileQuitItem;
GtkWidget *imageSizeItem, *imageRatioItem, *imageFreezeItem, *imageFramerateItem;
GtkWidget *synchroDisplayItem;
GtkWidget *helpAboutItem;
// StatusBar
GtkWidget *statusbar;
GtkWidget *fpsStatusBar;
GtkWidget *fpsStatusBar2;

guint timeout_ID=0;
guint timeout_update_ID=0;

ViewerResources _resources;


static void createObjects() {
    command=new string("");
}

static void deleteObjects() {
    /*if (ptr_inputPort!=0)
        delete ptr_inputPort;
    if (ptr_portCallback!=0)
        delete ptr_portCallback;*/
    delete command;
}

static double dec2current(int value) {
    double step = 4.5; // every bit is converted in 4.5pA
    int bits = 24;
    int bitvalue;
    double current = 0;
    for (int i = bits - 1 ; i >= 0 ; i--) {
        int mask = 1;
        for (int j = 0; j < i; j++) {
            mask *= 2;
        }
        if (mask & value) {
            bitvalue = 1;
        }
        else {
            bitvalue = 0;
        }
        current += bitvalue * mask * step;   
    }
    return current; //in pA
}


static double current2dec(double value) {
    double step = 4.5; // every bit is converted in 4.5pA
    double current = value / step;
    return current; //in decimal
}


//-------------------------------------------------
// Main Window Callbacks
//-------------------------------------------------
/**
* usual callback function 
*/
static void callback( GtkWidget *widget,gpointer   data ){
    printf ("Hello again - %s was pressed \n", (char *) data);
    
    if(!strcmp((char *)data,"Buttonl")){
        printf("Button1");
        command->assign("set b1");
    }
    if(!strcmp((char *)data,"Button2")){
        printf("Button2");
        command->assign("set b2");
    }
    if(!strcmp((char *)data,"Button3")){
        printf("Button3");
        command->assign("set b3");
    }
    if(!strcmp((char *)data,"Button4")){
        printf("Button4");
        command->assign("set b4");
    }
    if(!strcmp((char *)data,"Button5")){
        printf("b5");
        command->assign("set ipp");
    }

}

static void enter_callbackSYTH( GtkWidget *widget, GtkWidget *entry ) {
    const gchar *entry_text;
    entry_text = gtk_entry_get_text (GTK_ENTRY (entry));
    printf ("Entry contents :%s \n",entry_text);
    double current =g_ascii_strtod(entry_text,NULL);
    gtk_adjustment_set_value((GtkAdjustment*)adjSYTH,current2dec(current));
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_SYTH);
        bot.addInt((int) ((GtkAdjustment*)adjSYTH)->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
}

static void enter_callbackSYTA( GtkWidget *widget, GtkWidget *entry ) {
    const gchar *entry_text;
    entry_text = gtk_entry_get_text (GTK_ENTRY (entry));
    printf ("Entry contents  : %s\n", entry_text);
    double current =g_ascii_strtod(entry_text,NULL);
    gtk_adjustment_set_value((GtkAdjustment*)adjSYTA,current2dec(current));
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_SYTA);
        bot.addInt((int) ((GtkAdjustment*)adjSYTA)->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
}

static void enter_callbackSYPA( GtkWidget *widget, GtkWidget *entry ) {
    const gchar *entry_text;
    entry_text = gtk_entry_get_text (GTK_ENTRY (entry));
    printf ("Entry contents  : %s\n", entry_text);
    double current =g_ascii_strtod(entry_text,NULL);
    gtk_adjustment_set_value((GtkAdjustment*)adjSYPA,current2dec(current));
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_SYPA);
        bot.addInt((int) ((GtkAdjustment*)adjSYPA)->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
}

static void enter_callbackSYPH( GtkWidget *widget, GtkWidget *entry ) {
    const gchar *entry_text;
    entry_text = gtk_entry_get_text (GTK_ENTRY (entry));
    printf ("Entry contents : %s\n", entry_text);
    double current =g_ascii_strtod(entry_text,NULL);
    gtk_adjustment_set_value((GtkAdjustment*)adjSYPH,current2dec(current));
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_SYPH);
        bot.addInt((int) ((GtkAdjustment*)adjSYPH)->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
}

static void enter_callbackTPB( GtkWidget *widget, GtkWidget *entry ) {
    const gchar *entry_text;
    entry_text = gtk_entry_get_text (GTK_ENTRY (entry));
    printf ("Entry contents  : %s\n", entry_text);
    double current =g_ascii_strtod(entry_text,NULL);
    gtk_adjustment_set_value((GtkAdjustment*)adjTPB,current2dec(current));
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_TPB);
        bot.addInt((int) ((GtkAdjustment*)adjTPB)->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
}

static void enter_callbackCDR( GtkWidget *widget, GtkWidget *entry ) {
    const gchar *entry_text;
    entry_text = gtk_entry_get_text (GTK_ENTRY (entry));
    printf ("Entry contents  : %s\n", entry_text);
    double current =g_ascii_strtod(entry_text,NULL);
    gtk_adjustment_set_value((GtkAdjustment*)adjCDR,current2dec(current));
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_CDR);
        bot.addInt((int) ((GtkAdjustment*)adjCDR)->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
}

static void enter_callbackCDS( GtkWidget *widget, GtkWidget *entry ) {
    const gchar *entry_text;
    entry_text = gtk_entry_get_text (GTK_ENTRY (entry));
    printf ("Entry contents  : %s\n", entry_text);
    double current =g_ascii_strtod(entry_text,NULL);
    gtk_adjustment_set_value((GtkAdjustment*)adjCDS,current2dec(current));
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_CDS);
        bot.addInt((int) ((GtkAdjustment*)adjCDS)->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
}

static void enter_callbackCDP( GtkWidget *widget, GtkWidget *entry ) {
    const gchar *entry_text;
    entry_text = gtk_entry_get_text (GTK_ENTRY (entry));
    printf ("Entry contents  : %s\n", entry_text);
    double current =g_ascii_strtod(entry_text,NULL);
    gtk_adjustment_set_value((GtkAdjustment*)adjCDP,current2dec(current));
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_CDP);
        bot.addInt((int) ((GtkAdjustment*)adjCDP)->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
}

static void enter_callbackRPX( GtkWidget *widget, GtkWidget *entry ) {
    const gchar *entry_text;
    entry_text = gtk_entry_get_text (GTK_ENTRY (entry));
    printf ("Entry contents  : %s\n", entry_text);
    double current =g_ascii_strtod(entry_text,NULL);
    gtk_adjustment_set_value((GtkAdjustment*)adjRPX,current2dec(current));
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_RPX);
        bot.addInt((int) ((GtkAdjustment*)adjRPX)->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
}

static void enter_callbackRPY( GtkWidget *widget, GtkWidget *entry ) {
    const gchar *entry_text;
    entry_text = gtk_entry_get_text (GTK_ENTRY (entry));
    printf ("Entry contents  : %s\n", entry_text);
    double current =g_ascii_strtod(entry_text,NULL);
    gtk_adjustment_set_value((GtkAdjustment*)adjRPY,current2dec(current));
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_RPY);
        bot.addInt((int) ((GtkAdjustment*)adjRPY)->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
}

static void enter_callbackIFR( GtkWidget *widget, GtkWidget *entry ) {
    const gchar *entry_text;
    entry_text = gtk_entry_get_text (GTK_ENTRY (entry));
    printf ("Entry contents  : %s\n", entry_text);
    double current =g_ascii_strtod(entry_text,NULL);
    gtk_adjustment_set_value((GtkAdjustment*)adjIFR,current2dec(current));
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_IFR);
        bot.addInt((int) ((GtkAdjustment*)adjIFR)->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
}

static void enter_callbackIFT( GtkWidget *widget, GtkWidget *entry ) {
    const gchar *entry_text;
    entry_text = gtk_entry_get_text (GTK_ENTRY (entry));
    printf ("Entry contents  : %s\n", entry_text);
    double current =g_ascii_strtod(entry_text,NULL);
    gtk_adjustment_set_value((GtkAdjustment*)adjIFT,current2dec(current));
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_IFT);
        bot.addInt((int) ((GtkAdjustment*)adjIFT)->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
}

static void enter_callbackIFL( GtkWidget *widget, GtkWidget *entry ) {
    const gchar *entry_text;
    entry_text = gtk_entry_get_text (GTK_ENTRY (entry));
    printf ("Entry contents  : %s\n", entry_text);
    double current =g_ascii_strtod(entry_text,NULL);
    gtk_adjustment_set_value((GtkAdjustment*)adjIFL,current2dec(current));
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_IFL);
        bot.addInt((int) ((GtkAdjustment*)adjIFL)->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
}

static void enter_callbackCDOF( GtkWidget *widget, GtkWidget *entry ) {
    const gchar *entry_text;
    entry_text = gtk_entry_get_text (GTK_ENTRY (entry));
    printf ("Entry contents  : %s\n", entry_text);
    double current =g_ascii_strtod(entry_text,NULL);
    gtk_adjustment_set_value((GtkAdjustment*)adjCDOF,current2dec(current));
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_CDOF);
        bot.addInt((int) ((GtkAdjustment*)adjCDOF)->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
}

static void enter_callbackSYPW( GtkWidget *widget, GtkWidget *entry ) {
    const gchar *entry_text;
    entry_text = gtk_entry_get_text (GTK_ENTRY (entry));
    printf ("Entry contents  : %s\n", entry_text);
    double current =g_ascii_strtod(entry_text,NULL);
    gtk_adjustment_set_value((GtkAdjustment*)adjSYPW,current2dec(current));
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_SYPW);
        bot.addInt((int) ((GtkAdjustment*)adjSYPW)->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
}

static void enter_callbackSYW( GtkWidget *widget, GtkWidget *entry ) {
    const gchar *entry_text;
    entry_text = gtk_entry_get_text (GTK_ENTRY (entry));
    printf ("Entry contents  : %s\n", entry_text);
    double current =g_ascii_strtod(entry_text,NULL);
    gtk_adjustment_set_value((GtkAdjustment*)adjSYW,current2dec(current));
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_SYW);
        bot.addInt((int) ((GtkAdjustment*)adjSYW)->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
}

static void enter_callbackCDON( GtkWidget *widget, GtkWidget *entry ) {
    const gchar *entry_text;
    entry_text = gtk_entry_get_text (GTK_ENTRY (entry));
    printf ("Entry contents  : %s\n", entry_text);
    double current =g_ascii_strtod(entry_text,NULL);
    gtk_adjustment_set_value((GtkAdjustment*)adjCDON,current2dec(current));
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_CDON);
        bot.addInt((int) ((GtkAdjustment*)adjCDON)->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
}

static void enter_callbackCDD( GtkWidget *widget, GtkWidget *entry ) {
    const gchar *entry_text;
    entry_text = gtk_entry_get_text (GTK_ENTRY (entry));
    printf ("Entry contents  : %s\n", entry_text);
    double current =g_ascii_strtod(entry_text,NULL);
    gtk_adjustment_set_value((GtkAdjustment*)adjCDD,current2dec(current));
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_CDD);
        bot.addInt((int) ((GtkAdjustment*)adjCDD)->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
}

static void enter_callbackEMCH( GtkWidget *widget, GtkWidget *entry ) {
    const gchar *entry_text;
    entry_text = gtk_entry_get_text (GTK_ENTRY (entry));
    printf ("Entry contents  : %s\n", entry_text);
    double current =g_ascii_strtod(entry_text,NULL);
    gtk_adjustment_set_value((GtkAdjustment*)adjEMCH,current2dec(current));
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_EMCH);
        bot.addInt((int) ((GtkAdjustment*)adjEMCH)->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
}

static void enter_callbackEMCT( GtkWidget *widget, GtkWidget *entry ) {
    const gchar *entry_text;
    entry_text = gtk_entry_get_text (GTK_ENTRY (entry));
    printf ("Entry contents  : %s\n", entry_text);
    double current =g_ascii_strtod(entry_text,NULL);
    gtk_adjustment_set_value((GtkAdjustment*)adjEMCT,current2dec(current));
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_EMCT);
        bot.addInt((int) ((GtkAdjustment*)adjEMCT)->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
}

static void enter_callbackCDI( GtkWidget *widget, GtkWidget *entry ) {
    const gchar *entry_text;
    entry_text = gtk_entry_get_text (GTK_ENTRY (entry));
    printf ("Entry contents  : %s\n", entry_text);
    double current =g_ascii_strtod(entry_text,NULL);
    gtk_adjustment_set_value((GtkAdjustment*)adjCDI,current2dec(current));
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_CDI);
        bot.addInt((int) ((GtkAdjustment*)adjCDI)->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
}

static void enter_callbackCDRG( GtkWidget *widget, GtkWidget *entry ) {
    const gchar *entry_text;
    entry_text = gtk_entry_get_text (GTK_ENTRY (entry));
    printf ("Entry contents  : %s\n", entry_text);
    double current =g_ascii_strtod(entry_text,NULL);
    gtk_adjustment_set_value((GtkAdjustment*)adjCDRG,current2dec(current));
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_CDRG);
        bot.addInt((int) ((GtkAdjustment*)adjCDRG)->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
}

static void enter_callbackSELF( GtkWidget *widget, GtkWidget *entry ) {
    const gchar *entry_text;
    entry_text = gtk_entry_get_text (GTK_ENTRY (entry));
    printf ("Entry contents  : %s\n", entry_text);
    double current =g_ascii_strtod(entry_text,NULL);
    gtk_adjustment_set_value((GtkAdjustment*)adjSELF,current2dec(current));
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_SELF);
        bot.addInt((int) ((GtkAdjustment*)adjSELF)->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
}

static void enter_callbackFOLL( GtkWidget *widget, GtkWidget *entry ) {
    const gchar *entry_text;
    entry_text = gtk_entry_get_text (GTK_ENTRY (entry));
    printf ("Entry contents  : %s\n", entry_text);
    double current =g_ascii_strtod(entry_text,NULL);
    gtk_adjustment_set_value((GtkAdjustment*)adjFOLL,current2dec(current));
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_FOLL);
        bot.addInt((int) ((GtkAdjustment*)adjFOLL)->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
}

static void enter_callbackARBP( GtkWidget *widget, GtkWidget *entry ) {
    const gchar *entry_text;
    entry_text = gtk_entry_get_text (GTK_ENTRY (entry));
    printf ("Entry contents  : %s\n", entry_text);
    double current =g_ascii_strtod(entry_text,NULL);
    gtk_adjustment_set_value((GtkAdjustment*)adjARBP,current2dec(current));
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_ARBP);
        bot.addInt((int) ((GtkAdjustment*)adjARBP)->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
}

static void enter_callbackEMVL( GtkWidget *widget, GtkWidget *entry ) {
    const gchar *entry_text;
    entry_text = gtk_entry_get_text (GTK_ENTRY (entry));
    printf ("Entry contents  : %s\n", entry_text);
    double current =g_ascii_strtod(entry_text,NULL);
    gtk_adjustment_set_value((GtkAdjustment*)adjEMVL,current2dec(current));
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_EMVL);
        bot.addInt((int) ((GtkAdjustment*)adjEMVL)->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
}

static void enter_callbackCDC( GtkWidget *widget, GtkWidget *entry ) {
    const gchar *entry_text;
    entry_text = gtk_entry_get_text (GTK_ENTRY (entry));
    printf ("Entry contents  : %s\n", entry_text);
    double current = g_ascii_strtod(entry_text,NULL);
    gtk_adjustment_set_value((GtkAdjustment*)adjCDC,current2dec(current));
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_CDC);
        bot.addInt((int) ((GtkAdjustment*)adjCDC)->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
}

static void enter_callbackEMVH( GtkWidget *widget, GtkWidget *entry ) {
    const gchar *entry_text;
    entry_text = gtk_entry_get_text (GTK_ENTRY (entry));
    printf ("Entry contents  : %s\n", entry_text);
    double current =g_ascii_strtod(entry_text,NULL);
    gtk_adjustment_set_value((GtkAdjustment*)adjEMVH,current2dec(current));
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_EMVH);
        bot.addInt((int) ((GtkAdjustment*)adjEMVH)->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
}

static void enter_callbackSAVELOAD( GtkWidget *widget, GtkWidget *entry ) {
    printf(" changed the name of the file \n");
    entry_file = gtk_entry_get_text (GTK_ENTRY (entry));   
    printf ("Entry contents SAVE & LOAD  : %s\n", entry_file);
}

/*static void entry_toggle_editable( GtkWidget *checkbutton, GtkWidget *entry ) {
    gtk_editable_set_editable (GTK_EDITABLE (entry),
                               GTK_TOGGLE_BUTTON (checkbutton)->active);
}

static void entry_toggle_visibility( GtkWidget *checkbutton, GtkWidget *entry ) {
    gtk_entry_set_visibility (GTK_ENTRY (entry),
                              GTK_TOGGLE_BUTTON (checkbutton)->active);
}
*/




//-------------------------------------------------
// Call Backs Left
//-------------------------------------------------
static void cb_digits_SynThr( GtkAdjustment *adj ) {
    if (_pOutPort!=NULL) {
        Bottle in;
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_SYTH);
        bot.addInt((int) adj->value);
        //_pOutPort->Content() = _outBottle;
        _pOutPort->write(bot,in);
    }
    gchar* tpoint = new gchar();
    g_ascii_dtostr(tpoint,24,dec2current((int) adj->value));
    gtk_entry_set_text (GTK_ENTRY (entrySYTH), tpoint);
}

static void cb_digits_SynTau( GtkAdjustment *adj ) {
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_SYTA);
        bot.addInt((int) adj->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
    gchar* tpoint = new gchar();
    g_ascii_dtostr(tpoint,24,dec2current((int) adj->value));
    gtk_entry_set_text (GTK_ENTRY (entrySYTA), tpoint);
}

static void cb_digits_SynPxlTau( GtkAdjustment *adj ) {
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_SYPA);
        bot.addInt((int) adj->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
    gchar* tpoint = new gchar();
    g_ascii_dtostr(tpoint,24,dec2current((int) adj->value));
    gtk_entry_set_text (GTK_ENTRY (entrySYPA), tpoint);
}

static void cb_digits_SynPxlThr( GtkAdjustment *adj ) {
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_SYPH);
        bot.addInt((int) adj->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
    gchar* tpoint = new gchar();
    g_ascii_dtostr(tpoint,24,dec2current((int) adj->value));
    gtk_entry_set_text (GTK_ENTRY (entrySYPH), tpoint);
}

static void cb_digits_testPbias( GtkAdjustment *adj ) {
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_TPB);
        bot.addInt((int) adj->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
    gchar* tpoint = new gchar();
    g_ascii_dtostr(tpoint,24,dec2current((int) adj->value));
    gtk_entry_set_text (GTK_ENTRY (entryTPB), tpoint);
}

static void cb_digits_CDRefr( GtkAdjustment *adj ) {
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_CDR);
        bot.addInt((int) adj->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
    gchar* tpoint = new gchar();
    g_ascii_dtostr(tpoint,24,dec2current((int) adj->value));
    gtk_entry_set_text (GTK_ENTRY (entryCDR), tpoint);
}

static void cb_digits_CDSf( GtkAdjustment *adj ) {
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_CDS);
        bot.addInt((int) adj->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
    gchar* tpoint = new gchar();
    g_ascii_dtostr(tpoint,24,dec2current((int) adj->value));
    gtk_entry_set_text (GTK_ENTRY (entryCDS), tpoint);
}

static void cb_digits_CDPr( GtkAdjustment *adj ) {
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_CDP);
        bot.addInt((int) adj->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
    gchar* tpoint = new gchar();
    g_ascii_dtostr(tpoint,24,dec2current((int) adj->value));
    gtk_entry_set_text (GTK_ENTRY (entryCDP), tpoint);
}

static void cb_digits_ReqPuX( GtkAdjustment *adj ) {
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_RPX);
        bot.addInt((int) adj->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
    gchar* tpoint = new gchar();
    g_ascii_dtostr(tpoint,24,dec2current((int) adj->value));
    gtk_entry_set_text (GTK_ENTRY (entryRPX), tpoint);
}

static void cb_digits_ReqPuY( GtkAdjustment *adj ) {
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_RPY);
        bot.addInt((int) adj->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
    gchar* tpoint = new gchar();
    g_ascii_dtostr(tpoint,24,dec2current((int) adj->value));
    gtk_entry_set_text (GTK_ENTRY (entryRPY), tpoint);
}

static void cb_digits_IFRf( GtkAdjustment *adj ) {
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_IFR);
        bot.addInt((int) adj->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
    gchar* tpoint = new gchar();
    g_ascii_dtostr(tpoint,24,dec2current((int) adj->value));
    gtk_entry_set_text (GTK_ENTRY (entryIFR), tpoint);
}

static void cb_digits_IFThr( GtkAdjustment *adj ) {
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_IFT);
        bot.addInt((int) adj->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
    gchar* tpoint = new gchar();
    g_ascii_dtostr(tpoint,24,dec2current((int) adj->value));
    gtk_entry_set_text (GTK_ENTRY (entryIFT), tpoint);
}



static void cb_digits_IFLk( GtkAdjustment *adj ) {
    mutex.wait();
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_IFL);
        bot.addInt((int) adj->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
    mutex.post();
    gchar* tpoint = new gchar();
    g_ascii_dtostr(tpoint,24,dec2current((int) adj->value));
    gtk_entry_set_text (GTK_ENTRY (entryIFL), tpoint);
}

static void cb_digits_CDOffThr( GtkAdjustment *adj ) {
    mutex.wait();
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_CDOF);
        bot.addInt((int) adj->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
    mutex.post();
    gchar* tpoint = new gchar();
    g_ascii_dtostr(tpoint,24,dec2current((int) adj->value));
    gtk_entry_set_text (GTK_ENTRY (entryCDOF), tpoint);
}

static void cb_digits_SynPxlW( GtkAdjustment *adj ) {
    mutex.wait();
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_SYPW);
        bot.addInt((int) adj->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
    mutex.post();
    gchar* tpoint = new gchar();
    g_ascii_dtostr(tpoint,24,dec2current((int) adj->value));
    gtk_entry_set_text (GTK_ENTRY (entrySYPW), tpoint);
}

static void cb_digits_SynW( GtkAdjustment *adj ) {
    mutex.wait();
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_SYW);
        bot.addInt((int) adj->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
    mutex.post();
    gchar* tpoint = new gchar();
    g_ascii_dtostr(tpoint,24,dec2current((int) adj->value));
    gtk_entry_set_text (GTK_ENTRY (entrySYW), tpoint);
}


static void cb_digits_CDOnThr( GtkAdjustment *adj ) {
    mutex.wait();
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_CDON);
        bot.addInt((int) adj->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
    mutex.post();
    gchar* tpoint = new gchar();
    g_ascii_dtostr(tpoint,24,dec2current((int) adj->value));
    gtk_entry_set_text (GTK_ENTRY (entryCDON), tpoint);
}

static void cb_digits_CDDiff( GtkAdjustment *adj ) {
    mutex.wait();
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_CDD);
        bot.addInt((int) adj->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
    mutex.post();
    gchar* tpoint = new gchar();
    g_ascii_dtostr(tpoint,24,dec2current((int) adj->value));
    gtk_entry_set_text (GTK_ENTRY (entryCDD), tpoint);
}

static void cb_digits_EMCompH( GtkAdjustment *adj ) {
    mutex.wait();
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_EMCH);
        bot.addInt((int) adj->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
    mutex.post();
    gchar* tpoint = new gchar();
    g_ascii_dtostr(tpoint,24,dec2current((int) adj->value));
    gtk_entry_set_text (GTK_ENTRY (entryEMCH), tpoint);
}

static void cb_digits_EMCompT( GtkAdjustment *adj ) {
    mutex.wait();
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_EMCT);
        bot.addInt((int) adj->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
    mutex.post();
    gchar* tpoint = new gchar();
    g_ascii_dtostr(tpoint,24,dec2current((int) adj->value));
    gtk_entry_set_text (GTK_ENTRY (entryEMCT), tpoint);
}

static void cb_digits_CDIoff( GtkAdjustment *adj ) {
    mutex.wait();
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_CDI);
        bot.addInt((int) adj->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
    mutex.post();
    gchar* tpoint = new gchar();
    g_ascii_dtostr(tpoint,24,dec2current((int) adj->value));
    gtk_entry_set_text (GTK_ENTRY (entryCDI), tpoint);
}


static void cb_digits_CDRGnd( GtkAdjustment *adj ) {
    mutex.wait();
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_CDRG);
        bot.addInt((int) adj->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
    mutex.post();
    gchar* tpoint = new gchar();
    g_ascii_dtostr(tpoint,24,dec2current((int) adj->value));
    gtk_entry_set_text (GTK_ENTRY (entryCDRG), tpoint);
}

static void cb_digits_Self( GtkAdjustment *adj ) {
    mutex.wait();
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_SELF);
        bot.addInt((int) adj->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
    mutex.post();
    gchar* tpoint = new gchar();
    g_ascii_dtostr(tpoint,24,dec2current((int) adj->value));
    gtk_entry_set_text (GTK_ENTRY (entrySELF), tpoint);
}

static void cb_digits_Foll( GtkAdjustment *adj ) {
    mutex.wait();
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_FOLL);
        bot.addInt((int) adj->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
    mutex.post();
    gchar* tpoint = new gchar();
    g_ascii_dtostr(tpoint,24,dec2current((int) adj->value));
    gtk_entry_set_text (GTK_ENTRY (entryFOLL), tpoint);
}

static void cb_digits_ArbPd( GtkAdjustment *adj ) {
    mutex.wait();
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_ARBP);
        bot.addInt((int) adj->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
    mutex.post();
    gchar* tpoint = new gchar();
    g_ascii_dtostr(tpoint,24,dec2current((int) adj->value));
    gtk_entry_set_text (GTK_ENTRY (entryARBP), tpoint);
}

static void cb_digits_EMVrefL( GtkAdjustment *adj ) {
    mutex.wait();
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_EMVL);
        bot.addInt((int) adj->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
    mutex.post();
    gchar* tpoint = new gchar();
    g_ascii_dtostr(tpoint,24,dec2current((int) adj->value));
    gtk_entry_set_text (GTK_ENTRY (entryEMVL), tpoint);
}

static void cb_digits_CDCas( GtkAdjustment *adj ) {
    mutex.wait();
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_CDC);
        bot.addInt((int) adj->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
    mutex.post();
    gchar* tpoint = new gchar();
    g_ascii_dtostr(tpoint,24,dec2current((int) adj->value));
    gtk_entry_set_text (GTK_ENTRY (entryCDC), tpoint);
}

static void cb_digits_EMVrefH( GtkAdjustment *adj ) {
    mutex.wait();
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_EMVH);
        bot.addInt((int) adj->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
    mutex.post();
    gchar* tpoint = new gchar();
    g_ascii_dtostr(tpoint,24,dec2current((int) adj->value));
    gtk_entry_set_text (GTK_ENTRY (entryEMVH), tpoint);
}


// ----------------- CALL BACK RIGHT -------------------------------------------



// ---------------------------------------------------------------------------------


static void callbackSaveButton( GtkWidget *widget,gpointer data ) {
    printf ("Save button - %s was pressed\n", (char *) data);
    mutex.wait();
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SAVE);
        bot.addVocab(COMMAND_VOCAB_BIAS);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
    mutex.post();

    //saving in local file
    if(entry_file != NULL) {
        printf("trying to save the following file %s \n", entry_file);    
        fout = fopen(entry_file,"w");
        int value = gtk_adjustment_get_value((GtkAdjustment*)adjSYTH);
        fprintf(fout,"%d \n", value);
        value = gtk_adjustment_get_value((GtkAdjustment*)adjSYTA);
        fprintf(fout,"%d \n", value);
        value = gtk_adjustment_get_value((GtkAdjustment*)adjSYPA);
        fprintf(fout,"%d \n", value);
        value = gtk_adjustment_get_value((GtkAdjustment*)adjSYPH);
        fprintf(fout,"%d \n", value);
        value = gtk_adjustment_get_value((GtkAdjustment*)adjTPB);
        fprintf(fout,"%d \n", value);
        value = gtk_adjustment_get_value((GtkAdjustment*)adjCDR);
        fprintf(fout,"%d \n", value);
        value = gtk_adjustment_get_value((GtkAdjustment*)adjCDS);
        fprintf(fout,"%d \n", value);
        value = gtk_adjustment_get_value((GtkAdjustment*)adjCDP);
        fprintf(fout,"%d \n", value);
        value = gtk_adjustment_get_value((GtkAdjustment*)adjRPX);
        fprintf(fout,"%d \n", value);
        value = gtk_adjustment_get_value((GtkAdjustment*)adjRPY);
        fprintf(fout,"%d \n", value);
        value = gtk_adjustment_get_value((GtkAdjustment*)adjIFR);
        fprintf(fout,"%d \n", value);
        value = gtk_adjustment_get_value((GtkAdjustment*)adjIFT);
        fprintf(fout,"%d \n", value);
        value = gtk_adjustment_get_value((GtkAdjustment*)adjIFL);
        fprintf(fout,"%d \n", value);
        value = gtk_adjustment_get_value((GtkAdjustment*)adjCDOF);
        fprintf(fout,"%d \n", value);
        value = gtk_adjustment_get_value((GtkAdjustment*)adjSYPW);
        fprintf(fout,"%d \n", value);
        value = gtk_adjustment_get_value((GtkAdjustment*)adjSYW);
        fprintf(fout,"%d \n", value);
        value = gtk_adjustment_get_value((GtkAdjustment*)adjCDON);
        fprintf(fout,"%d \n", value);
        value = gtk_adjustment_get_value((GtkAdjustment*)adjCDD);
        fprintf(fout,"%d \n", value);
        value = gtk_adjustment_get_value((GtkAdjustment*)adjEMCH);
        fprintf(fout,"%d \n", value);
        value = gtk_adjustment_get_value((GtkAdjustment*)adjEMCT);
        fprintf(fout,"%d \n", value);
        value = gtk_adjustment_get_value((GtkAdjustment*)adjCDI);
        fprintf(fout,"%d \n", value);
        value = gtk_adjustment_get_value((GtkAdjustment*)adjCDRG);
        fprintf(fout,"%d \n", value);
        value = gtk_adjustment_get_value((GtkAdjustment*)adjSELF);
        fprintf(fout,"%d \n", value);
        value = gtk_adjustment_get_value((GtkAdjustment*)adjFOLL);
        fprintf(fout,"%d \n", value);
        value = gtk_adjustment_get_value((GtkAdjustment*)adjARBP);
        fprintf(fout,"%d \n", value);
        value = gtk_adjustment_get_value((GtkAdjustment*)adjEMVL);
        fprintf(fout,"%d \n", value);
        value = gtk_adjustment_get_value((GtkAdjustment*)adjCDC);
        fprintf(fout,"%d \n", value);
        value = gtk_adjustment_get_value((GtkAdjustment*)adjEMVH);
        fprintf(fout,"%d \n", value);
        fclose(fout);
        printf("biases file correctly saved \n");
    }
}

static void callbackLoadButton( GtkWidget *widget,gpointer data ) {
    printf ("Load button - %s was pressed\n", (char *) data);
    mutex.wait();
    c = 100;
    printf("c %d \n", c);
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_LOAD);
        bot.addVocab(COMMAND_VOCAB_BIAS);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
    mutex.post();
    
    //loading in local file
    if(entry_file!=NULL){
        //char str[80];
        int current;
        printf("trying to read from the %s \n", entry_file);
        fout = fopen(entry_file,"r");
        std::string str;
        printf("scanning the file \n");
        
        fscanf (fout, "%s", &str[0]);
        current = atoi(str.c_str());
        printf("biasSYTH %d \n",current);
        gtk_adjustment_set_value((GtkAdjustment*)adjSYTH,current);
        fscanf (fout, "%s", &str[0]);
        current = atoi(str.c_str());
        printf("biasSYTA %d \n",current);
        gtk_adjustment_set_value((GtkAdjustment*)adjSYTA,current);
        fscanf (fout, "%s", &str[0]);
        current = atoi(str.c_str());
        printf("biasSYPA %d \n",current);
        gtk_adjustment_set_value((GtkAdjustment*)adjSYPA,current);
        fscanf (fout, "%s", &str[0]);
        current = atoi(str.c_str());
        printf("biasSYPH %d \n",current);
        gtk_adjustment_set_value((GtkAdjustment*)adjSYPH,current);
         fscanf (fout, "%s", &str[0]);
        current = atoi(str.c_str());
        printf("biasTPB %d \n",current);
        gtk_adjustment_set_value((GtkAdjustment*)adjTPB,current);
        fscanf (fout, "%s", &str[0]);
        current = atoi(str.c_str());
        printf("biasCDR %d \n",current);
        gtk_adjustment_set_value((GtkAdjustment*)adjCDR,current);
        fscanf (fout, "%s", &str[0]);
        current = atoi(str.c_str());
        printf("biasCDS %d \n",current);
        gtk_adjustment_set_value((GtkAdjustment*)adjCDS,current);
        fscanf (fout, "%s", &str[0]);
        current = atoi(str.c_str());
        printf("biasCDP %d \n",current);
        gtk_adjustment_set_value((GtkAdjustment*)adjCDP,current);
         fscanf (fout, "%s", &str[0]);
        current = atoi(str.c_str());
        printf("biasRPX %d \n",current);
        gtk_adjustment_set_value((GtkAdjustment*)adjRPX,current);
        fscanf (fout, "%s", &str[0]);
        current = atoi(str.c_str());
        printf("biasRPY %d \n",current);
        gtk_adjustment_set_value((GtkAdjustment*)adjRPY,current);
        fscanf (fout, "%s", &str[0]);
        current = atoi(str.c_str());
        printf("biasIFR %d \n",current);
        gtk_adjustment_set_value((GtkAdjustment*)adjIFR,current);
        fscanf (fout, "%s", &str[0]);
        current = atoi(str.c_str());
        printf("biasIFT %d \n",current);
        gtk_adjustment_set_value((GtkAdjustment*)adjIFT,current);
         fscanf (fout, "%s", &str[0]);
        current = atoi(str.c_str());
        printf("biasIFL %d \n",current);
        gtk_adjustment_set_value((GtkAdjustment*)adjIFL,current);
        fscanf (fout, "%s", &str[0]);
        current = atoi(str.c_str());
        printf("biasCDOF %d \n",current);
        gtk_adjustment_set_value((GtkAdjustment*)adjCDOF,current);
        fscanf (fout, "%s", &str[0]);
        current = atoi(str.c_str());
        printf("biasSYPW %d \n",current);
        gtk_adjustment_set_value((GtkAdjustment*)adjSYPW,current);
        fscanf (fout, "%s", &str[0]);
        current = atoi(str.c_str());
        printf("biasSYW %d \n",current);
        gtk_adjustment_set_value((GtkAdjustment*)adjSYW,current);
         fscanf (fout, "%s", &str[0]);
        current = atoi(str.c_str());
        printf("biasCDON %d \n",current);
        gtk_adjustment_set_value((GtkAdjustment*)adjCDON,current);
        fscanf (fout, "%s", &str[0]);
        current = atoi(str.c_str());
        printf("biasCDD %d \n",current);
        gtk_adjustment_set_value((GtkAdjustment*)adjCDD,current);
        fscanf (fout, "%s", &str[0]);
        current = atoi(str.c_str());
        printf("biasEMCH %d \n",current);
        gtk_adjustment_set_value((GtkAdjustment*)adjEMCH,current);
        fscanf (fout, "%s", &str[0]);
        current = atoi(str.c_str());
        printf("biasEMCT %d \n",current);
        gtk_adjustment_set_value((GtkAdjustment*)adjEMCT,current);
         fscanf (fout, "%s", &str[0]);
        current = atoi(str.c_str());
        printf("biasCDI %d \n",current);
        gtk_adjustment_set_value((GtkAdjustment*)adjCDI,current);
        fscanf (fout, "%s", &str[0]);
        current = atoi(str.c_str());
        printf("biasCDRG %d \n",current);
        gtk_adjustment_set_value((GtkAdjustment*)adjCDRG,current);
        fscanf (fout, "%s", &str[0]);
        current = atoi(str.c_str());
        printf("biasSELF %d \n",current);
        gtk_adjustment_set_value((GtkAdjustment*)adjSELF,current);
        fscanf (fout, "%s", &str[0]);
        current = atoi(str.c_str());
        printf("biasFOLL %d \n",current);
        gtk_adjustment_set_value((GtkAdjustment*)adjFOLL,current);
         fscanf (fout, "%s", &str[0]);
        current = atoi(str.c_str());
        printf("biasARBP %d \n",current);
        gtk_adjustment_set_value((GtkAdjustment*)adjARBP,current);
        fscanf (fout, "%s", &str[0]);
        current = atoi(str.c_str());
        printf("biasEMVL %d \n",current);
        gtk_adjustment_set_value((GtkAdjustment*)adjEMVL,current);
        fscanf (fout, "%s", &str[0]);
        current = atoi(str.c_str());
        printf("biasCDC %d \n",current);
        gtk_adjustment_set_value((GtkAdjustment*)adjCDC,current);
        fscanf (fout, "%s", &str[0]);
        current = atoi(str.c_str());
        printf("biasEMVH %d \n",current);
        gtk_adjustment_set_value((GtkAdjustment*)adjEMVH,current);
        
        
        //char* pstr = &str[0];
        printf("successfully scanned! \n");
        fclose(fout); 
    }
}

static void callbackProgBiasButton( GtkWidget *widget,gpointer data ) {
    printf ("Prog Bias - %s was pressed\n", (char *) data);
    mutex.wait();
    c = -1;
    printf("c = -1 \n");
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_PROG);
        if(!strcmp((char*) data,"left")) {
            bot.addVocab(COMMAND_VOCAB_LEFT);
        }
        else {
            bot.addVocab(COMMAND_VOCAB_RIGHT);
        }

        bot.addVocab(COMMAND_VOCAB_BIAS);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
    mutex.post();
}

static void callbackDumpButton( GtkWidget *widget,gpointer data ) {
    printf ("Dump Button - %s was pressed\n", (char *) data);
    mutex.wait();
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_DUMP);
        if(!strcmp((char*) data,"on")) {
            bot.addVocab(COMMAND_VOCAB_ON);
        }
        else {
            bot.addVocab(COMMAND_VOCAB_OFF);
        }

        //bot.addVocab(COMMAND_VOCAB_BIAS);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
    mutex.post();
}


gint timeout_CB(gpointer data) {
    c++;
    //printf("c %d \n", c);
    switch(c) {
        case 0: {
            if(_pOutPort->getOutputCount()) {
                    yarp::os::Bottle bot; //= _pOutPort->prepare();
                    bot.clear();
                    bot.addVocab(COMMAND_VOCAB_GET);
                    bot.addVocab(COMMAND_VOCAB_SYTH);
                    //_pOutPort->Content() = _outBottle;
                    Bottle in;
                    _pOutPort->write(bot,in);
                    printf("       in bottle %s", in.toString().c_str());
                    double value=in.get(0).asDouble();
                    printf("getting SYTH %f \n", value);
                    //mutex.wait();
                    gtk_adjustment_set_value(GTK_ADJUSTMENT (adjSYTH),value);
                    //mutex.post();
            }
        }
        break;
        case 1: {
            if(_pOutPort->getOutputCount()) {
                    yarp::os::Bottle bot; //= _pOutPort->prepare();
                    bot.clear();
                    bot.addVocab(COMMAND_VOCAB_GET);
                    bot.addVocab(COMMAND_VOCAB_SYTA);
                    //_pOutPort->Content() = _outBottle;
                    Bottle in;
                    _pOutPort->write(bot,in);
                    double value=in.get(0).asDouble();
                    printf("getting SYTA %f", value);
                    //mutex.wait();
                    gtk_adjustment_set_value(GTK_ADJUSTMENT (adjSYTA),value);
                    //mutex.post();
            }
        }
        break;
        case 2: {
            if(_pOutPort->getOutputCount()) {
                    yarp::os::Bottle bot; //= _pOutPort->prepare();
                    bot.clear();
                    bot.addVocab(COMMAND_VOCAB_GET);
                    bot.addVocab(COMMAND_VOCAB_SYPA);
                    //_pOutPort->Content() = _outBottle;
                    Bottle in;
                    _pOutPort->write(bot,in);
                    double value=in.get(0).asDouble();
                    printf("getting SYPA %f \n", value);
                    //mutex.wait();
                    gtk_adjustment_set_value(GTK_ADJUSTMENT (adjSYPA),value);
                    //mutex.post();
            }
        }
        break;
        case 3: {
            if(_pOutPort->getOutputCount()) {
                    yarp::os::Bottle bot; //= _pOutPort->prepare();
                    bot.clear();
                    bot.addVocab(COMMAND_VOCAB_GET);
                    bot.addVocab(COMMAND_VOCAB_SYPH);
                    //_pOutPort->Content() = _outBottle;
                    Bottle in;
                    _pOutPort->write(bot,in);
                    double value=in.get(0).asDouble();
                    printf("getting SYPH %f", value);
                    //mutex.wait();
                    gtk_adjustment_set_value(GTK_ADJUSTMENT (adjSYPH),value);
                    //mutex.post();
            }
        }
        break;
        case 4: {
            if(_pOutPort->getOutputCount()) {
                    yarp::os::Bottle bot; //= _pOutPort->prepare();
                    bot.clear();
                    bot.addVocab(COMMAND_VOCAB_GET);
                    bot.addVocab(COMMAND_VOCAB_TPB);
                    //_pOutPort->Content() = _outBottle;
                    Bottle in;
                    _pOutPort->write(bot,in);
                    double value=in.get(0).asDouble();
                    printf("getting TPB %f \n", value);
                    //mutex.wait();
                    gtk_adjustment_set_value(GTK_ADJUSTMENT (adjTPB),value);
                    //mutex.post();
            }
        }
        break;
        case 5: {
            if(_pOutPort->getOutputCount()) {
                    yarp::os::Bottle bot; //= _pOutPort->prepare();
                    bot.clear();
                    bot.addVocab(COMMAND_VOCAB_GET);
                    bot.addVocab(COMMAND_VOCAB_CDR);
                    //_pOutPort->Content() = _out5Bottle;
                    Bottle in;
                    _pOutPort->write(bot,in);
                    double value=in.get(0).asDouble();
                    printf("getting CDR %f \n", value);
                    //mutex.wait();
                    gtk_adjustment_set_value(GTK_ADJUSTMENT (adjCDR),value);
                    //mutex.post();
            }
        }
        break;
        case 6: {
            if(_pOutPort->getOutputCount()) {
                    yarp::os::Bottle bot; //= _pOutPort->prepare();
                    bot.clear();
                    bot.addVocab(COMMAND_VOCAB_GET);
                    bot.addVocab(COMMAND_VOCAB_CDS);
                    //_pOutPort->Content() = _outBottle;
                    Bottle in;
                    _pOutPort->write(bot,in);
                    double value=in.get(0).asDouble();
                    printf("getting CDS %f \n", value);
                    //mutex.wait();
                    gtk_adjustment_set_value(GTK_ADJUSTMENT (adjCDS),value);
                    //mutex.post();
           }
        }
        break;
        case 7: {
            if(_pOutPort->getOutputCount()) {
                    yarp::os::Bottle bot; //= _pOutPort->prepare();
                    bot.clear();
                    bot.addVocab(COMMAND_VOCAB_GET);
                    bot.addVocab(COMMAND_VOCAB_CDP);
                    //_pOutPort->Content() = _outBottle;
                    Bottle in;
                    _pOutPort->write(bot,in);
                    double value=in.get(0).asDouble();
                    printf("getting CDP %f \n", value);
                    //mutex.wait();
                    gtk_adjustment_set_value(GTK_ADJUSTMENT (adjCDP),value);
                    //mutex.post();
            }
        }
        break;
        case 8: {
            if(_pOutPort->getOutputCount()) {
                    yarp::os::Bottle bot; //= _pOutPort->prepare();
                    bot.clear();
                    bot.addVocab(COMMAND_VOCAB_GET);
                    bot.addVocab(COMMAND_VOCAB_RPX);
                    //_pOutPort->Content() = _outBottle;
                    Bottle in;
                    _pOutPort->write(bot,in);
                    double value=in.get(0).asDouble();
                    printf("getting RPX  %f \n", value);
                    //mutex.wait();
                    gtk_adjustment_set_value(GTK_ADJUSTMENT (adjRPX),value);
                    //mutex.post();
            }
        }
        break;
        case 9: {
            if(_pOutPort->getOutputCount()) {
                    yarp::os::Bottle bot; //= _pOutPort->prepare();
                    bot.clear();
                    bot.addVocab(COMMAND_VOCAB_GET);
                    bot.addVocab(COMMAND_VOCAB_RPY);
                    //_pOutPort->Content() = _outBottle;
                    Bottle in;
                    _pOutPort->write(bot,in);
                    double value=in.get(0).asDouble();
                    printf("getting RPY %f \n", value);
                    //mutex.wait();
                    gtk_adjustment_set_value(GTK_ADJUSTMENT (adjRPY),value);
                    //mutex.post();
            }
        }
        break;
        case 10: {
            if(_pOutPort->getOutputCount()) {
                    yarp::os::Bottle bot; //= _pOutPort->prepare();
                    bot.clear();
                    bot.addVocab(COMMAND_VOCAB_GET);
                    bot.addVocab(COMMAND_VOCAB_IFR);
                    //_pOutPort->Content() = _outBottle;
                    Bottle in;
                    _pOutPort->write(bot,in);
                    double value=in.get(0).asDouble();
                    printf("getting IFR %f \n", value);
                    //mutex.wait();
                    gtk_adjustment_set_value(GTK_ADJUSTMENT (adjIFR),value);
                    //mutex.post();
            }
        }
        break;
        case 11: {
            if(_pOutPort->getOutputCount()) {
                    yarp::os::Bottle bot; //= _pOutPort->prepare();
                    bot.clear();
                    bot.addVocab(COMMAND_VOCAB_GET);
                    bot.addVocab(COMMAND_VOCAB_IFT);
                    //_pOutPort->Content() = _out5Bottle;
                    Bottle in;
                    _pOutPort->write(bot,in);
                    double value=in.get(0).asDouble();
                    printf("getting IFT %f \n", value);
                    //mutex.wait();
                    gtk_adjustment_set_value(GTK_ADJUSTMENT (adjIFT),value);
                    //mutex.post();
            }
        }
        break;
        case 12: {
            if(_pOutPort->getOutputCount()) {
                    yarp::os::Bottle bot; //= _pOutPort->prepare();
                    bot.clear();
                    bot.addVocab(COMMAND_VOCAB_GET);
                    bot.addVocab(COMMAND_VOCAB_IFL);
                    //_pOutPort->Content() = _outBottle;
                    Bottle in;
                    _pOutPort->write(bot,in);
                    double value=in.get(0).asDouble();
                    printf("getting IFL %f \n", value);
                    //mutex.wait();
                    gtk_adjustment_set_value(GTK_ADJUSTMENT (adjIFL),value);
                    //mutex.post();
            }
        }
        break;
        case 13: {
            if(_pOutPort->getOutputCount()) {
                    yarp::os::Bottle bot; //= _pOutPort->prepare();
                    bot.clear();
                    bot.addVocab(COMMAND_VOCAB_GET);
                    bot.addVocab(COMMAND_VOCAB_CDOF);
                    //_pOutPort->Content() = _outBottle;
                    Bottle in;
                    _pOutPort->write(bot,in);
                    double value=in.get(0).asDouble();
                    printf("getting CDOF %f \n", value);
                    //mutex.wait();
                    gtk_adjustment_set_value(GTK_ADJUSTMENT (adjCDOF),value);
                    //mutex.post();
            }
        }
        break;
        case 14: {
            if(_pOutPort->getOutputCount()) {
                    yarp::os::Bottle bot; //= _pOutPort->prepare();
                    bot.clear();
                    bot.addVocab(COMMAND_VOCAB_GET);
                    bot.addVocab(COMMAND_VOCAB_SYPW);
                    //_pOutPort->Content() = _outBottle;
                    Bottle in;
                    _pOutPort->write(bot,in);
                    double value=in.get(0).asDouble();
                    printf("getting SYPW %f \n", value);
                    //mutex.wait();
                    gtk_adjustment_set_value(GTK_ADJUSTMENT (adjSYPW),value);
                    //mutex.post();
            }
        }
        break;
        case 15: {
            if(_pOutPort->getOutputCount()) {
                    yarp::os::Bottle bot; //= _pOutPort->prepare();
                    bot.clear();
                    bot.addVocab(COMMAND_VOCAB_GET);
                    bot.addVocab(COMMAND_VOCAB_SYW);
                    //_pOutPort->Content() = _outBottle;
                    Bottle in;
                    _pOutPort->write(bot,in);
                    double value=in.get(0).asDouble();
                    printf("getting SYW %f \n", value);
                    //mutex.wait();
                    gtk_adjustment_set_value(GTK_ADJUSTMENT (adjSYW),value);
                    //mutex.post();
            }
        }
        break;
        case 16: {
            if(_pOutPort->getOutputCount()) {
                    yarp::os::Bottle bot; //= _pOutPort->prepare();
                    bot.clear();
                    bot.addVocab(COMMAND_VOCAB_GET);
                    bot.addVocab(COMMAND_VOCAB_CDON);
                    //_pOutPort->Content() = _outBottle;
                    Bottle in;
                    _pOutPort->write(bot,in);
                    double value=in.get(0).asDouble();
                    printf("getting CDON %f \n", value);
                    //mutex.wait();
                    gtk_adjustment_set_value(GTK_ADJUSTMENT (adjCDON),value);
                    //mutex.post();
            }
        }
        break;
        case 17: {
            if(_pOutPort->getOutputCount()) {
                    yarp::os::Bottle bot; //= _pOutPort->prepare();
                    bot.clear();
                    bot.addVocab(COMMAND_VOCAB_GET);
                    bot.addVocab(COMMAND_VOCAB_CDD);
                    //_pOutPort->Content() = _outBottle;
                    Bottle in;
                    _pOutPort->write(bot,in);
                    double value=in.get(0).asDouble();
                    printf("getting CDD %f \n", value);
                    //mutex.wait();
                    gtk_adjustment_set_value(GTK_ADJUSTMENT (adjCDD),value);
                    //mutex.post();
            }
        }
        break;
        case 18: {
            if(_pOutPort->getOutputCount()) {
                    yarp::os::Bottle bot; //= _pOutPort->prepare();
                    bot.clear();
                    bot.addVocab(COMMAND_VOCAB_GET);
                    bot.addVocab(COMMAND_VOCAB_EMCH);
                    //_pOutPort->Content() = _outBottle;
                    Bottle in;
                    _pOutPort->write(bot,in);
                    double value=in.get(0).asDouble();
                    printf("getting EMCH %f \n", value);
                    //mutex.wait();
                    gtk_adjustment_set_value(GTK_ADJUSTMENT (adjEMCH),value);
                    //mutex.post();
            }
        }
        break;
        case 19: {
            if(_pOutPort->getOutputCount()) {
                    yarp::os::Bottle bot; //= _pOutPort->prepare();
                    bot.clear();
                    bot.addVocab(COMMAND_VOCAB_GET);
                    bot.addVocab(COMMAND_VOCAB_EMCT);
                    //_pOutPort->Content() = _outBottle;
                    Bottle in;
                    _pOutPort->write(bot,in);
                    double value=in.get(0).asDouble();
                    printf("getting EMCT %f \n", value);
                    //mutex.wait();
                    gtk_adjustment_set_value(GTK_ADJUSTMENT (adjEMCT),value);
                    //mutex.post();
            }
        }
        break;
        case 20: {
            if(_pOutPort->getOutputCount()) {
                    yarp::os::Bottle bot; //= _pOutPort->prepare();
                    bot.clear();
                    bot.addVocab(COMMAND_VOCAB_GET);
                    bot.addVocab(COMMAND_VOCAB_CDI);
                    //_pOutPort->Content() = _outBottle;
                    Bottle in;
                    _pOutPort->write(bot,in);
                    double value=in.get(0).asDouble();
                    printf("getting CDI %f \n", value);
                    //mutex.wait();
                    gtk_adjustment_set_value(GTK_ADJUSTMENT (adjCDI),value);
                    //mutex.post();
            }
        }
        break;
        case 21: {
            if(_pOutPort->getOutputCount()) {
                    yarp::os::Bottle bot; //= _pOutPort->prepare();
                    bot.clear();
                    bot.addVocab(COMMAND_VOCAB_GET);
                    bot.addVocab(COMMAND_VOCAB_CDRG);
                    //_pOutPort->Content() = _outBottle;
                    Bottle in;
                    _pOutPort->write(bot,in);
                    double value=in.get(0).asDouble();
                    printf("getting CDRG %f \n", value);
                    //mutex.wait();
                    gtk_adjustment_set_value(GTK_ADJUSTMENT (adjCDRG),value);
                    //mutex.post();
            }
        }
        break;
        case 22: {
            if(_pOutPort->getOutputCount()) {
                    yarp::os::Bottle bot; //= _pOutPort->prepare();
                    bot.clear();
                    bot.addVocab(COMMAND_VOCAB_GET);
                    bot.addVocab(COMMAND_VOCAB_SELF);
                    //_pOutPort->Content() = _outBottle;
                    Bottle in;
                    _pOutPort->write(bot,in);
                    double value=in.get(0).asDouble();
                    printf("getting SELF %f \n", value);
                    //mutex.wait();
                    gtk_adjustment_set_value(GTK_ADJUSTMENT (adjSELF),value);
                    //mutex.post();
            }
        }
        break;
        case 23: {
            if(_pOutPort->getOutputCount()) {
                    yarp::os::Bottle bot; //= _pOutPort->prepare();
                    bot.clear();
                    bot.addVocab(COMMAND_VOCAB_GET);
                    bot.addVocab(COMMAND_VOCAB_FOLL);
                    //_pOutPort->Content() = _outBottle;
                    Bottle in;
                    _pOutPort->write(bot,in);
                    double value=in.get(0).asDouble();
                    printf("getting FOLL %f \n", value);
                    //mutex.wait();
                    gtk_adjustment_set_value(GTK_ADJUSTMENT (adjFOLL),value);
                    //mutex.post();
            }
        }
        break;
        case 24: {
            if(_pOutPort->getOutputCount()) {
                    yarp::os::Bottle bot; //= _pOutPort->prepare();
                    bot.clear();
                    bot.addVocab(COMMAND_VOCAB_GET);
                    bot.addVocab(COMMAND_VOCAB_ARBP);
                    //_pOutPort->Content() = _outBottle;
                    Bottle in;
                    _pOutPort->write(bot,in);
                    double value=in.get(0).asDouble();
                    printf("getting ARPB %f \n", value);
                    //mutex.wait();
                    gtk_adjustment_set_value(GTK_ADJUSTMENT (adjARBP),value);
                    //mutex.post();
            }
        }
        break;
        case 25: {
            if(_pOutPort->getOutputCount()) {
                    yarp::os::Bottle bot; //= _pOutPort->prepare();
                    bot.clear();
                    bot.addVocab(COMMAND_VOCAB_GET);
                    bot.addVocab(COMMAND_VOCAB_EMVL);
                    //_pOutPort->Content() = _outBottle;
                    Bottle in;
                    _pOutPort->write(bot,in);
                    double value=in.get(0).asDouble();
                    printf("getting EMVL %f \n", value);
                    //mutex.wait();
                    gtk_adjustment_set_value(GTK_ADJUSTMENT (adjEMVL),value);
                    //mutex.post();
            }
        }
        break;
        case 26: {
            if(_pOutPort->getOutputCount()) {
                    yarp::os::Bottle bot; //= _pOutPort->prepare();
                    bot.clear();
                    bot.addVocab(COMMAND_VOCAB_GET);
                    bot.addVocab(COMMAND_VOCAB_CDC);
                    //_pOutPort->Content() = _outBottle;
                    Bottle in;
                    _pOutPort->write(bot,in);
                    double value=in.get(0).asDouble();
                    printf("getting CDC %f \n", value);
                    //mutex.wait();
                    gtk_adjustment_set_value(GTK_ADJUSTMENT (adjCDC),value);
                    //mutex.post();
            }
        }
        break;
        case 27: {
            if(_pOutPort->getOutputCount()) {
                    yarp::os::Bottle bot; //= _pOutPort->prepare();
                    bot.clear();
                    bot.addVocab(COMMAND_VOCAB_GET);
                    bot.addVocab(COMMAND_VOCAB_EMVH);
                    //_pOutPort->Content() = _outBottle;
                    Bottle in;
                    _pOutPort->write(bot,in);
                    double value=in.get(0).asDouble();
                    printf("getting EMVH %f \n", value);
                    //mutex.wait();
                    gtk_adjustment_set_value(GTK_ADJUSTMENT (adjEMVH),value);
                    
                    //mutex.post();
            }
        }        break;
        default: {
                    c=-1;
                 }
    }

    //portFpsData.getStats(av, min, max);
    //portFpsData.reset();
    gchar *msg;
    //gdk_threads_enter();
    msg=g_strdup_printf("selectiveAttentionInterface");
    updateStatusbar(fpsStatusBar, msg);
    g_free(msg);
    //displayFpsData.getStats(av, min, max);
    //displayFpsData.reset();
    
    //periodToFreq(av, min, max, avHz, minHz, maxHz);

    //msg=g_strdup_printf("Display: %.1f (min:%.1f max:%.1f) fps", avHz, minHz, maxHz);
    //updateStatusbar(fpsStatusBar2, msg);
    //g_free(msg);
    //gdk_threads_leave();
    return TRUE;
}

//gint timeout_CB (gpointer data) {
//    c++;
//    return TRUE;
//}


gboolean delete_event( GtkWidget *widget, GdkEvent *event, gpointer data ) {
    // If you return FALSE in the "delete_event" signal handler,
    // GTK will emit the "destroy" signal. Returning TRUE means
    // you don't want the window to be destroyed.
    // This is useful for popping up 'are you sure you want to quit?'
    // type dialogs. 
    cleanExit();
    return TRUE;
}

gint expose_CB (GtkWidget *widget, GdkEventExpose *event, gpointer data) {
    return TRUE;
}

static gboolean configure_event( GtkWidget *widget, GdkEventConfigure *event ) {
   _resources.configure(widget, 
        widget->allocation.width,
        widget->allocation.height);
  return TRUE;
}
gint menuFileQuit_CB(GtkWidget *widget, gpointer data) {
    cleanExit();
    return TRUE;
}

gint menuHelpAbout_CB(GtkWidget *widget, gpointer data) {
#if GTK_CHECK_VERSION(2,6,0)
    const gchar *authors[] = 
        {
            "Yarp developers",
            NULL
        };
    const gchar *license =
        "Released under the terms of the LGPLv2.1 or later, see LGPL.TXT\n"
        "The complete license description is contained in the\n"
        "COPYING file included in this distribution.\n"
        "Please refer to this file for complete\n"
        "information about the licensing of YARP.\n"
        "\n"
        "DISCLAIMERS: LICENSOR WARRANTS THAT THE COPYRIGHT IN AND TO THE\n"
        "SOFTWARE IS OWNED BY THE LICENSOR OR THAT THE SOFTWARE IS\n"
        "DISTRIBUTED BY LICENSOR UNDER A VALID CURRENT LICENSE. EXCEPT AS\n"
        "EXPRESSLY STATED IN THE IMMEDIATELY PRECEDING SENTENCE, THE\n"
        "SOFTWARE IS PROVIDED BY THE LICENSOR, CONTRIBUTORS AND COPYRIGHT\n"
        "OWNERS AS IS, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED\n"
        "INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,\n"
        "FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. IN NO\n"
        "EVENT SHALL THE LICENSOR, CONTRIBUTORS OR COPYRIGHT OWNERS BE\n"
        "LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN\n"
        "ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN\n"
        "ONNECTION WITH THE SOFTWARE.\n";

    gtk_show_about_dialog(GTK_WINDOW(mainWindow),
                          "name", "selAttentionInterface",
                          "version", "1.0",
                          "license", license,
                          "website", "http://sourceforge.net/projects/yarp0",
                          "comments", "Interface for selectiveAttentionEngine",
                          "authors", authors,
                          NULL);
#else
    printf("Missing functionality on older GTK version, sorry\n");
#endif

    return TRUE;
}

gint menuFileSingle_CB(GtkWidget *widget, GdkEventExpose *event, gpointer data) {
    if ( gtk_check_menu_item_get_active (GTK_CHECK_MENU_ITEM(widget)) ) 
        {
            gtk_check_menu_item_set_active(GTK_CHECK_MENU_ITEM(fileSetItem), FALSE);
            gtk_widget_show_all (saveSingleDialog);
        } 
    else 
        {
            gtk_widget_hide (saveSingleDialog);
        }

    return TRUE;
}
void setTimedMode(guint dT) {
   // ptr_portCallback->mustDraw(false);
    if (timeout_ID!=0)
        gtk_timeout_remove(timeout_ID);
    timeout_ID = gtk_timeout_add (dT, timeout_CB, NULL);
}

void setSynchroMode() {
    gtk_timeout_remove(timeout_ID);
    timeout_ID=0;
    //ptr_portCallback->mustDraw(true);
}

//-------------------------------------------------
// Non Modal Dialogs
//-------------------------------------------------
GtkWidget* createSaveSingleDialog(void) {

    GtkWidget *dialog = NULL;
    GtkWidget *hbox;
    GtkWidget *button;
    dialog = gtk_dialog_new ();
    gtk_window_set_title(GTK_WINDOW(dialog), "Save Snapshot");
    gtk_window_set_modal(GTK_WINDOW(dialog), FALSE);
    gtk_window_set_transient_for(GTK_WINDOW(dialog), GTK_WINDOW(mainWindow));
    //gtk_window_resize(GTK_WINDOW(dialog), 185, 40);
    gtk_window_set_resizable(GTK_WINDOW(dialog), FALSE);
    //gtk_window_set_default_size(GTK_WINDOW(dialog), 185, 40);
    gtk_window_set_destroy_with_parent(GTK_WINDOW(dialog), TRUE);
    gtk_dialog_set_has_separator (GTK_DIALOG(dialog), FALSE);
    hbox = gtk_hbox_new (TRUE, 8); // parameters (gboolean homogeneous_space, gint spacing);
    button = gtk_button_new_from_stock(GTK_STOCK_SAVE);
    gtk_widget_set_size_request (GTK_WIDGET(button), 150,50);
    gtk_box_pack_start (GTK_BOX (hbox), button, TRUE, TRUE, 16); // parameters (GtkBox *box, GtkWidget *child, gboolean expand, gboolean fill, guint padding);
    gtk_box_pack_start (GTK_BOX (GTK_DIALOG (dialog)->vbox), hbox, FALSE, FALSE, 8); // parameters (GtkBox *box, GtkWidget *child, gboolean expand, gboolean fill, guint padding);
    
    //gtk_container_set_border_width(GTK_CONTAINER(hbox), 5);
    
    return dialog;
}

GtkWidget* createSaveSetDialog(void) {
    GtkWidget *dialog = NULL;
    dialog = gtk_dialog_new ();
    gtk_window_set_title(GTK_WINDOW(dialog), "Save Image Set");
    gtk_window_set_modal(GTK_WINDOW(dialog), FALSE);
    gtk_window_set_transient_for(GTK_WINDOW(dialog), GTK_WINDOW(mainWindow));
    gtk_window_set_resizable(GTK_WINDOW(dialog), FALSE);
    //gtk_window_set_default_size(GTK_WINDOW(dialog), 190, 40);
    gtk_window_set_destroy_with_parent(GTK_WINDOW(dialog), TRUE);
    gtk_dialog_set_has_separator (GTK_DIALOG(dialog), FALSE);
    return dialog;
}

//-------------------------------------------------
// Main Window Menubar
//-------------------------------------------------
GtkWidget* createMenubar(void) {
    GtkWidget *menubar;

    menubar =  gtk_menu_bar_new ();
    GtkWidget *menuSeparator;	
    // Submenus Items on menubar
    fileItem = gtk_menu_item_new_with_label ("File");
    helpItem = gtk_menu_item_new_with_label ("Help");
    // Submenu: File 
    fileMenu = gtk_menu_new();
    fileSingleItem = gtk_check_menu_item_new_with_label ("Save single image..");
    gtk_menu_append( GTK_MENU(fileMenu), fileSingleItem);
    gtk_signal_connect( GTK_OBJECT(fileSingleItem), "toggled", GTK_SIGNAL_FUNC(menuFileSingle_CB), mainWindow);
    fileSetItem = gtk_check_menu_item_new_with_label ("Save a set of images..");
    gtk_menu_append( GTK_MENU(fileMenu), fileSetItem);
    menuSeparator = gtk_separator_menu_item_new();
    gtk_menu_append( GTK_MENU(fileMenu), menuSeparator);
    fileQuitItem = gtk_menu_item_new_with_label ("Quit");
    gtk_menu_append( GTK_MENU(fileMenu), fileQuitItem);
    gtk_signal_connect( GTK_OBJECT(fileQuitItem), "activate", GTK_SIGNAL_FUNC(menuFileQuit_CB), mainWindow);
    // Submenu: Image  
    imageMenu = gtk_menu_new();
    
    menuSeparator = gtk_separator_menu_item_new();
    gtk_menu_append( GTK_MENU(imageMenu), menuSeparator);
    menuSeparator = gtk_separator_menu_item_new();
    gtk_menu_append( GTK_MENU(imageMenu), menuSeparator);
    
    // Submenu: Help
    helpMenu = gtk_menu_new();
    helpAboutItem = gtk_menu_item_new_with_label ("About..");
    gtk_menu_append( GTK_MENU(helpMenu), helpAboutItem);
    gtk_signal_connect( GTK_OBJECT(helpAboutItem), "activate", GTK_SIGNAL_FUNC(menuHelpAbout_CB), mainWindow);
    // linking the submenus to items on menubar
    gtk_menu_item_set_submenu(GTK_MENU_ITEM(fileItem), fileMenu);
    gtk_menu_item_set_submenu(GTK_MENU_ITEM(helpItem), helpMenu);
    // appending the submenus to the menubar
    gtk_menu_bar_append(GTK_MENU_BAR(menubar), fileItem);
    gtk_menu_item_set_right_justified (GTK_MENU_ITEM (helpItem), TRUE);
    gtk_menu_bar_append(GTK_MENU_BAR(menubar), helpItem);
  
    return menubar;
}

static GtkWidget *xpm_label_box( gchar     *xpm_filename,gchar *label_text ) {
    GtkWidget *box;
    GtkWidget *label;
    GtkWidget *image;

    /* Create box for image and label */
    box = gtk_hbox_new (FALSE, 0);
    gtk_container_set_border_width (GTK_CONTAINER (box), 2);

    /* Now on to the image stuff */
    if(xpm_filename!=NULL)
        image = gtk_image_new_from_file (xpm_filename);

    /* Create a label for the button */
    label = gtk_label_new (label_text);

    /* Pack the image and label into the box */
    if(xpm_filename!=NULL)
        gtk_box_pack_start (GTK_BOX (box), image, FALSE, FALSE, 3);
    gtk_box_pack_start (GTK_BOX (box), label, FALSE, FALSE, 3);

    if(xpm_filename!=NULL)
        gtk_widget_show (image);
    gtk_widget_show (label);

    return box;
}

static void scale_set_default_values( GtkScale *scale ) {
    gtk_range_set_update_policy (GTK_RANGE (scale),GTK_UPDATE_CONTINUOUS);
    gtk_scale_set_digits (scale, 2);
    gtk_scale_set_value_pos (scale, GTK_POS_TOP);
    gtk_scale_set_draw_value (scale, TRUE);
}

//-------------------------------------------------
// Main Window Statusbar
//-------------------------------------------------
void updateStatusbar(GtkWidget *statusbar, gchar *msg) {
    GtkStatusbar *sb=GTK_STATUSBAR (statusbar);

    //gtk_statusbar_pop (sb, 0); // clear any previous message, underflow is allowed 
    //gtk_statusbar_push (sb, 0, msg);
}

//-------------------------------------------------
// Main Window 
//-------------------------------------------------
GtkWidget* createMainWindow(void) {
    
    
    GtkRequisition actualSize;
    GtkWidget* window;
    
    //gtk_init (&argc, &argv);
    window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title (GTK_WINDOW (window), "selectiveAttentionInterface");
    gtk_window_set_default_size(GTK_WINDOW (window), 205, 300); 
    gtk_window_set_resizable (GTK_WINDOW (window), TRUE);
    g_signal_connect (G_OBJECT (window), "destroy",
                      G_CALLBACK (gtk_main_quit),
                      NULL);

    // When the window is given the "delete_event" signal (this is given
    // by the window manager, usually by the "close" option, or on the
    // titlebar), we ask it to call the delete_event () function
    // as defined above. The data passed to the callback
    // function is NULL and is ignored in the callback function.
    //g_signal_connect (G_OBJECT (window), "delete_event", G_CALLBACK (delete_event), NULL);
    
    // Box for main window
    GtkWidget *buttonSave,*buttonLoad, *buttonProgBias, *buttonDumpOn, *buttonDumpOff;
    GtkWidget *boxButton;
    GtkWidget *box, *box2, *box3, *box4, *box5, *box6, *box7;
    // init text boxes
    gchar* tpoint = new gchar();
    gint tmp_pos;

    box = gtk_vbox_new (FALSE, 0); // parameters (gboolean homogeneous_space, gint spacing);
    gtk_container_add (GTK_CONTAINER (window), box);
    // MenuBar for main window
    menubar = createMenubar();
    gtk_box_pack_start (GTK_BOX (box), menubar, FALSE, TRUE, 0); // parameters (GtkBox *box, GtkWidget *child, gboolean expand, gboolean fill, guint padding);
    //gtk_widget_size_request(menubar, &actualSize);
    // Drawing Area : here the image will be drawed
    //da = gtk_drawing_area_new ();
    //g_signal_connect (da, "expose_event", G_CALLBACK (expose_CB), NULL);
    /*if (_options.outputEnabled == 1)
        {
            g_signal_connect (da, "button_press_event", G_CALLBACK (clickDA_CB), NULL);
            // Ask to receive events the drawing area doesn't normally subscribe to
            gtk_widget_set_events (da, gtk_widget_get_events (da) | GDK_BUTTON_PRESS_MASK);
        }*/
    //gtk_box_pack_start(GTK_BOX(box), da, TRUE, TRUE, 0);
    
    
    //Toolbox area
    //creates the area as collection of port processes sequence
    box2 = gtk_vbox_new (FALSE, 0); // parameters (gboolean homogeneous_space, gint spacing);
    gtk_container_add (GTK_CONTAINER (window), box2);
    GtkWidget *boxButtons;
    GtkWidget *boxSliders;
    boxButtons = gtk_vbox_new (FALSE, 0); // parameters (gboolean homogeneous_space, gint spacing);
    gtk_container_set_border_width (GTK_CONTAINER (boxButtons), 0);
    boxSliders = gtk_hbox_new (TRUE, 0); // parameters (gboolean homogeneous_space, gint spacing);
    gtk_container_set_border_width (GTK_CONTAINER (boxSliders), 0);
    
    //-----SCALE section
    GtkWidget *scrollbar;
    GtkWidget *separator;
    GtkWidget *label;
    
    //GtkWidget *scale;
    
    
    GtkWidget *hscale, *vscale;

    // value, lower, upper, step_increment, page_increment, page_size 
    //adj1 = gtk_adjustment_new (0.0, 0.0, 1.0, 0.01, 1.0, 1.0);
    //vscale = gtk_vscale_new (GTK_ADJUSTMENT (adj1));
    //scale_set_default_values (GTK_SCALE (vscale));
    //gtk_box_pack_start (GTK_BOX (boxSliders), vscale, TRUE, TRUE, 0);
    //gtk_widget_show (vscale);

    //separator = gtk_vseparator_new ();
    //gtk_box_pack_start (GTK_BOX (boxSliders), separator, FALSE, FALSE, 0);
    //gtk_widget_show (separator);

    //----------BOX3 SECTION:1
    //box3 is the single area that controls the processing towards the output port
    //every processes sequence has a sequence of checkboxes a label and a button
    box3 = gtk_hbox_new (FALSE, 0);
    gtk_container_set_border_width (GTK_CONTAINER (box3), 0);
    gtk_box_pack_start (GTK_BOX (box2), box3, FALSE, FALSE, 0);
    gtk_widget_show (box3);

    //----------BOX5 SUBSECTION:1
    box5 = gtk_vbox_new (FALSE, 0);
    gtk_container_set_border_width (GTK_CONTAINER (box5), 0);

    label = gtk_label_new ("BIAS CD:");
    gtk_box_pack_start (GTK_BOX (box5), label, FALSE, FALSE, 0);
    gtk_widget_show (label);

    label = gtk_label_new ("CDSf");
    gtk_box_pack_start (GTK_BOX (box5), label, FALSE, FALSE, 0);
    gtk_widget_show (label);
    
    //gtk_entry_set_text (GTK_ENTRY (entry), "hello");
    entryCDS = gtk_entry_new ();
    gtk_entry_set_max_length (GTK_ENTRY (entryCDS), 50);
    g_signal_connect (entryCDS, "activate",
		      G_CALLBACK (enter_callbackCDS),
		      entryCDS);
    tmp_pos = GTK_ENTRY (entryCDS)->text_length;
    
    g_ascii_dtostr(tpoint,24,dec2current(CDS_DEFAULT_VALUE));
    gtk_editable_insert_text (GTK_EDITABLE (entryCDS), tpoint , -1, &tmp_pos);
    gtk_editable_select_region (GTK_EDITABLE (entryCDS),
			        0, GTK_ENTRY (entryCDS)->text_length);
    gtk_box_pack_start (GTK_BOX (box5), entryCDS, TRUE, TRUE, 0);
    gtk_widget_show (entryCDS);
    
    adjCDS = gtk_adjustment_new (CDS_DEFAULT_VALUE, 0,16777215, 10, 1000, 0);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adjCDS));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box5), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adjCDS), "value_changed",
                      G_CALLBACK (cb_digits_CDSf), NULL);
    
    label = gtk_label_new ("CDCas");
    gtk_box_pack_start (GTK_BOX (box5), label, FALSE, FALSE, 0);
    gtk_widget_show (label);

    entryCDC = gtk_entry_new ();
    gtk_entry_set_max_length (GTK_ENTRY (entryCDC), 50);
    g_signal_connect (entryCDC, "activate",
		      G_CALLBACK (enter_callbackCDC),
		      entryCDC);
    tmp_pos = GTK_ENTRY (entryCDC)->text_length;
    g_ascii_dtostr(tpoint,24,dec2current(CDC_DEFAULT_VALUE));
    gtk_editable_insert_text (GTK_EDITABLE (entryCDC), tpoint , -1, &tmp_pos);
    gtk_editable_select_region (GTK_EDITABLE (entryCDC),
			        0, GTK_ENTRY (entryCDC)->text_length);
    gtk_box_pack_start (GTK_BOX (box5), entryCDC, TRUE, TRUE, 0);
    gtk_widget_show (entryCDC);
    
    adjCDC = gtk_adjustment_new (CDC_DEFAULT_VALUE, 0,16777215, 10, 1000, 0);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adjCDC));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box5), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adjCDC), "value_changed",
                      G_CALLBACK (cb_digits_CDCas), NULL);


    label = gtk_label_new ("CDPr");
    gtk_box_pack_start (GTK_BOX (box5), label, FALSE, FALSE, 0);
    gtk_widget_show (label);

    entryCDP = gtk_entry_new ();
    gtk_entry_set_max_length (GTK_ENTRY (entryCDP), 50);
    g_signal_connect (entryCDP, "activate",
		      G_CALLBACK (enter_callbackCDP),
		      entryCDP);
    tmp_pos = GTK_ENTRY (entryCDP)->text_length;
    g_ascii_dtostr(tpoint,24,dec2current(CDP_DEFAULT_VALUE));
    gtk_editable_insert_text (GTK_EDITABLE (entryCDP), tpoint , -1, &tmp_pos);
    gtk_editable_select_region (GTK_EDITABLE (entryCDP),
			        0, GTK_ENTRY (entryCDP)->text_length);
    gtk_box_pack_start (GTK_BOX (box5), entryCDP, TRUE, TRUE, 0);
    gtk_widget_show (entryCDP);
    
    adjCDP = gtk_adjustment_new (CDP_DEFAULT_VALUE, 0,16777215, 10, 1000, 0);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adjCDP));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box5), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adjCDP), "value_changed",
                      G_CALLBACK (cb_digits_CDPr), NULL);

    label = gtk_label_new ("CDRefr");
    gtk_box_pack_start (GTK_BOX (box5), label, FALSE, FALSE, 0);
    gtk_widget_show (label);

    entryCDR = gtk_entry_new ();
    gtk_entry_set_max_length (GTK_ENTRY (entryCDR), 50);
    g_signal_connect (entryCDR, "activate",
		      G_CALLBACK (enter_callbackCDR),
		      entryCDR);
    tmp_pos = GTK_ENTRY (entryCDR)->text_length;
    g_ascii_dtostr(tpoint,24,dec2current(CDR_DEFAULT_VALUE));
    gtk_editable_insert_text (GTK_EDITABLE (entryCDR), tpoint , -1, &tmp_pos);
    gtk_editable_select_region (GTK_EDITABLE (entryCDR),
			        0, GTK_ENTRY (entryCDR)->text_length);
    gtk_box_pack_start (GTK_BOX (box5), entryCDR, TRUE, TRUE, 0);
    gtk_widget_show (entryCDR);
    
    adjCDR = gtk_adjustment_new (CDR_DEFAULT_VALUE, 0,16777215, 10 , 1000, 0);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adjCDR));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box5), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adjCDR), "value_changed",
                      G_CALLBACK (cb_digits_CDRefr), NULL);

    label = gtk_label_new ("CDRGnd");
    gtk_box_pack_start (GTK_BOX (box5), label, FALSE, FALSE, 0);
    gtk_widget_show (label);

    entryCDRG = gtk_entry_new ();
    gtk_entry_set_max_length (GTK_ENTRY (entryCDRG), 50);
    g_signal_connect (entryCDRG, "activate",
		      G_CALLBACK (enter_callbackCDRG),
		      entryCDRG);
    tmp_pos = GTK_ENTRY (entryCDRG)->text_length;
    g_ascii_dtostr(tpoint,24,dec2current(CDRG_DEFAULT_VALUE));
    gtk_editable_insert_text (GTK_EDITABLE (entryCDRG), tpoint , -1, &tmp_pos);
    gtk_editable_select_region (GTK_EDITABLE (entryCDRG),
			        0, GTK_ENTRY (entryCDRG)->text_length);
    gtk_box_pack_start (GTK_BOX (box5), entryCDRG, TRUE, TRUE, 0);
    gtk_widget_show (entryCDRG);
    
    adjCDRG = gtk_adjustment_new (CDRG_DEFAULT_VALUE, 0,16777215, 10, 1000, 0);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adjCDRG));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box5), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adjCDRG), "value_changed",
                      G_CALLBACK (cb_digits_CDRGnd), NULL);

    label = gtk_label_new ("CDOnThr");
    gtk_box_pack_start (GTK_BOX (box5), label, FALSE, FALSE, 0);
    gtk_widget_show (label);

    entryCDON = gtk_entry_new ();
    gtk_entry_set_max_length (GTK_ENTRY (entryCDON), 50);
    g_signal_connect (entryCDON, "activate",
		      G_CALLBACK (enter_callbackCDON),
		      entryCDON);
    tmp_pos = GTK_ENTRY (entryCDON)->text_length;
    g_ascii_dtostr(tpoint,24,dec2current(CDON_DEFAULT_VALUE));
    gtk_editable_insert_text (GTK_EDITABLE (entryCDON), tpoint , -1, &tmp_pos);
    gtk_editable_select_region (GTK_EDITABLE (entryCDON),
			        0, GTK_ENTRY (entryCDON)->text_length);
    gtk_box_pack_start (GTK_BOX (box5), entryCDON, TRUE, TRUE, 0);
    gtk_widget_show (entryCDON);
    
    adjCDON = gtk_adjustment_new (CDON_DEFAULT_VALUE, 0,16777215,10, 1000, 0);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adjCDON));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box5), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adjCDON), "value_changed",
                      G_CALLBACK (cb_digits_CDOnThr), NULL);

    label = gtk_label_new ("CDOffThr");
    gtk_box_pack_start (GTK_BOX (box5), label, FALSE, FALSE, 0);
    gtk_widget_show (label);

    entryCDOF = gtk_entry_new ();
    gtk_entry_set_max_length (GTK_ENTRY (entryCDOF), 50);
    g_signal_connect (entryCDOF, "activate",
		      G_CALLBACK (enter_callbackCDOF),
		      entryCDOF);
    tmp_pos = GTK_ENTRY (entryCDOF)->text_length;
    g_ascii_dtostr(tpoint,24,dec2current(CDOF_DEFAULT_VALUE));
    gtk_editable_insert_text (GTK_EDITABLE (entryCDOF), tpoint , -1, &tmp_pos);
    gtk_editable_select_region (GTK_EDITABLE (entryCDOF),
			        0, GTK_ENTRY (entryCDOF)->text_length);
    gtk_box_pack_start (GTK_BOX (box5), entryCDOF, TRUE, TRUE, 0);
    gtk_widget_show (entryCDOF);
    
    adjCDOF = gtk_adjustment_new (CDOF_DEFAULT_VALUE, 0,16777215, 10, 1000, 0);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adjCDOF));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box5), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adjCDOF), "value_changed",
                      G_CALLBACK (cb_digits_CDOffThr), NULL);
    
    label = gtk_label_new ("CDDiff");
    gtk_box_pack_start (GTK_BOX (box5), label, FALSE, FALSE, 0);
    gtk_widget_show (label);

    entryCDD = gtk_entry_new ();
    gtk_entry_set_max_length (GTK_ENTRY (entryCDD), 50);
    g_signal_connect (entryCDD, "activate",
		      G_CALLBACK (enter_callbackCDD),
		      entryCDD);
    tmp_pos = GTK_ENTRY (entryCDD)->text_length;
    g_ascii_dtostr(tpoint,24,dec2current(CDD_DEFAULT_VALUE));
    gtk_editable_insert_text (GTK_EDITABLE (entryCDD), tpoint , -1, &tmp_pos);
    gtk_editable_select_region (GTK_EDITABLE (entryCDD),
			        0, GTK_ENTRY (entryCDD)->text_length);
    gtk_box_pack_start (GTK_BOX (box5), entryCDD, TRUE, TRUE, 0);
    gtk_widget_show (entryCDD);
    
    adjCDD = gtk_adjustment_new (CDD_DEFAULT_VALUE, 0.0,16777215, 10, 1000, 0);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adjCDD));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box5), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adjCDD), "value_changed",
                      G_CALLBACK (cb_digits_CDDiff), NULL);

    label = gtk_label_new ("CDIoff");
    gtk_box_pack_start (GTK_BOX (box5), label, FALSE, FALSE, 0);
    gtk_widget_show (label);

    entryCDI = gtk_entry_new ();
    gtk_entry_set_max_length (GTK_ENTRY (entryCDI), 50);
    g_signal_connect (entryCDI, "activate",
		      G_CALLBACK (enter_callbackCDI),
		      entryCDI);
    tmp_pos = GTK_ENTRY (entryCDI)->text_length;
    g_ascii_dtostr(tpoint,24,dec2current(CDI_DEFAULT_VALUE));
    gtk_editable_insert_text (GTK_EDITABLE (entryCDI), tpoint , -1, &tmp_pos);
    gtk_editable_select_region (GTK_EDITABLE (entryCDI),
			        0, GTK_ENTRY (entryCDI)->text_length);
    gtk_box_pack_start (GTK_BOX (box5), entryCDI, TRUE, TRUE, 0);
    gtk_widget_show (entryCDI);
    
    adjCDI = gtk_adjustment_new (CDI_DEFAULT_VALUE, 0,16777215, 10, 1000, 0);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adjCDI));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box5), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adjCDI), "value_changed",
                      G_CALLBACK (cb_digits_CDIoff), NULL);

    label = gtk_label_new ("self");
    gtk_box_pack_start (GTK_BOX (box5), label, FALSE, FALSE, 0);
    gtk_widget_show (label);

    entrySELF = gtk_entry_new ();
    gtk_entry_set_max_length (GTK_ENTRY (entrySELF), 50);
    g_signal_connect (entrySELF, "activate",
		      G_CALLBACK (enter_callbackSELF),
		      entrySELF);
    tmp_pos = GTK_ENTRY (entrySELF)->text_length;
    g_ascii_dtostr(tpoint,24,dec2current(SELF_DEFAULT_VALUE));
    gtk_editable_insert_text (GTK_EDITABLE (entrySELF), tpoint , -1, &tmp_pos);
    gtk_editable_select_region (GTK_EDITABLE (entrySELF),
			        0, GTK_ENTRY (entrySELF)->text_length);
    gtk_box_pack_start (GTK_BOX (box5), entrySELF, TRUE, TRUE, 0);
    gtk_widget_show (entrySELF);
    
    adjSELF = gtk_adjustment_new (SELF_DEFAULT_VALUE, 0,16777215, 10, 1000, 0);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adjSELF));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box5), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adjSELF), "value_changed",
                      G_CALLBACK (cb_digits_Self), NULL);

    gtk_box_pack_start (GTK_BOX (box3), box5, FALSE, FALSE, 0);
    gtk_widget_show (box5);

    separator = gtk_vseparator_new ();
    gtk_box_pack_start (GTK_BOX (box3), separator, FALSE, TRUE, 10);
    gtk_widget_show (separator);

    // ------------ BOX 5 SUBSECTION 2
    box5 = gtk_vbox_new (FALSE, 0);
    gtk_container_set_border_width (GTK_CONTAINER (box5), 0);

    label = gtk_label_new ("BIAS IF :");
    gtk_box_pack_start (GTK_BOX (box5), label, FALSE, FALSE, 0);
    gtk_widget_show (label);

    label = gtk_label_new ("IFRf");
    gtk_box_pack_start (GTK_BOX (box5), label, FALSE, FALSE, 0);
    gtk_widget_show (label);

    entryIFR = gtk_entry_new ();
    gtk_entry_set_max_length (GTK_ENTRY (entryIFR), 50);
    g_signal_connect (entryIFR, "activate",
		      G_CALLBACK (enter_callbackIFR),
		      entryIFR);
    tmp_pos = GTK_ENTRY (entryIFR)->text_length;
    g_ascii_dtostr(tpoint,24,dec2current(IFR_DEFAULT_VALUE));
    gtk_editable_insert_text (GTK_EDITABLE (entryIFR), tpoint , -1, &tmp_pos);
    gtk_editable_select_region (GTK_EDITABLE (entryIFR),
			        0, GTK_ENTRY (entryIFR)->text_length);
    gtk_box_pack_start (GTK_BOX (box5), entryIFR, TRUE, TRUE, 0);
    gtk_widget_show (entryIFR);
    
    adjIFR = gtk_adjustment_new (IFR_DEFAULT_VALUE, 0,16777215,10000, 0, 0);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adjIFR));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box5), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adjIFR), "value_changed",
                      G_CALLBACK (cb_digits_IFRf), NULL);

    label = gtk_label_new ("IFThr");
    gtk_box_pack_start (GTK_BOX (box5), label, FALSE, FALSE, 0);
    gtk_widget_show (label);

    entryIFT = gtk_entry_new ();
    gtk_entry_set_max_length (GTK_ENTRY (entryIFT), 50);
    g_signal_connect (entryIFT, "activate",
		      G_CALLBACK (enter_callbackIFT),
		      entryIFT);
    tmp_pos = GTK_ENTRY (entryIFT)->text_length;
    g_ascii_dtostr(tpoint,24,dec2current(IFT_DEFAULT_VALUE));
    gtk_editable_insert_text (GTK_EDITABLE (entryIFT), tpoint , -1, &tmp_pos);
    gtk_editable_select_region (GTK_EDITABLE (entryIFT),
			        0, GTK_ENTRY (entryIFT)->text_length);
    gtk_box_pack_start (GTK_BOX (box5), entryIFT, TRUE, TRUE, 0);
    gtk_widget_show (entryIFT);
    
    adjIFT = gtk_adjustment_new (IFT_DEFAULT_VALUE, 0,16777215, 10, 1000, 0);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adjIFT));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box5), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adjIFT), "value_changed",
                      G_CALLBACK (cb_digits_IFThr), NULL);

    label = gtk_label_new ("IFLk");
    gtk_box_pack_start (GTK_BOX (box5), label, FALSE, FALSE, 0);
    gtk_widget_show (label);

    entryIFL = gtk_entry_new ();
    gtk_entry_set_max_length (GTK_ENTRY (entryIFL), 50);
    g_signal_connect (entryIFL, "activate",
		      G_CALLBACK (enter_callbackIFL),
		      entryIFL);
    tmp_pos = GTK_ENTRY (entryIFL)->text_length;
    g_ascii_dtostr(tpoint,24,dec2current(IFL_DEFAULT_VALUE));
    gtk_editable_insert_text (GTK_EDITABLE (entryIFL), tpoint , -1, &tmp_pos);
    gtk_editable_select_region (GTK_EDITABLE (entryIFL),
			        0, GTK_ENTRY (entryIFL)->text_length);
    gtk_box_pack_start (GTK_BOX (box5), entryIFL, TRUE, TRUE, 0);
    gtk_widget_show (entryIFL);
    
    adjIFL = gtk_adjustment_new (IFL_DEFAULT_VALUE, 0,16777215, 10, 1000, 0);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adjIFL));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box5), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adjIFL), "value_changed",
                      G_CALLBACK (cb_digits_IFLk), NULL);
    
    separator = gtk_hseparator_new ();
    gtk_box_pack_start (GTK_BOX (box5), separator, FALSE, TRUE, 10);
    gtk_widget_show (separator);

    label = gtk_label_new ("BIAS SYNPXL :");
    gtk_box_pack_start (GTK_BOX (box5), label, FALSE, FALSE, 0);
    gtk_widget_show (label);

    label = gtk_label_new ("SynPxlTau");
    gtk_box_pack_start (GTK_BOX (box5), label, FALSE, FALSE, 0);
    gtk_widget_show (label);

    entrySYPA = gtk_entry_new ();
    gtk_entry_set_max_length (GTK_ENTRY (entrySYPA), 50);
    g_signal_connect (entrySYPA, "activate",
		      G_CALLBACK (enter_callbackSYPA),
		      entrySYPA);
    tmp_pos = GTK_ENTRY (entrySYPA)->text_length;
    g_ascii_dtostr(tpoint,24,dec2current(SYPA_DEFAULT_VALUE));
    gtk_editable_insert_text (GTK_EDITABLE (entrySYPA), tpoint , -1, &tmp_pos);
    gtk_editable_select_region (GTK_EDITABLE (entrySYPA),
			        0, GTK_ENTRY (entrySYPA)->text_length);
    gtk_box_pack_start (GTK_BOX (box5), entrySYPA, TRUE, TRUE, 0);
    gtk_widget_show (entrySYPA);
    
    adjSYPA = gtk_adjustment_new (SYPA_DEFAULT_VALUE, 0,16777215, 10, 1000, 0);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adjSYPA));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box5), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adjSYPA), "value_changed",
                      G_CALLBACK (cb_digits_SynPxlTau), NULL);

    label = gtk_label_new ("SynPxlThr");
    gtk_box_pack_start (GTK_BOX (box5), label, FALSE, FALSE, 0);
    gtk_widget_show (label);

    entrySYPH = gtk_entry_new ();
    gtk_entry_set_max_length (GTK_ENTRY (entrySYPH), 50);
    g_signal_connect (entrySYPH, "activate",
		      G_CALLBACK (enter_callbackSYPH),
		      entrySYPH);
    tmp_pos = GTK_ENTRY (entrySYPH)->text_length;
    g_ascii_dtostr(tpoint,24,dec2current(SYPH_DEFAULT_VALUE));
    gtk_editable_insert_text (GTK_EDITABLE (entrySYPH), tpoint , -1, &tmp_pos);
    gtk_editable_select_region (GTK_EDITABLE (entrySYPH),
			        0, GTK_ENTRY (entrySYPH)->text_length);
    gtk_box_pack_start (GTK_BOX (box5), entrySYPH, TRUE, TRUE, 0);
    gtk_widget_show (entrySYPH);
    
    adjSYPH = gtk_adjustment_new (SYPH_DEFAULT_VALUE, 0,16777215, 10, 1000, 0);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adjSYPH));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box5), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adjSYPH), "value_changed",
                      G_CALLBACK (cb_digits_SynPxlThr), NULL);

    label = gtk_label_new ("SynPxlW");
    gtk_box_pack_start (GTK_BOX (box5), label, FALSE, FALSE, 0);
    gtk_widget_show (label);

    entrySYPW = gtk_entry_new ();
    gtk_entry_set_max_length (GTK_ENTRY (entrySYPW), 50);
    g_signal_connect (entrySYPW, "activate",
		      G_CALLBACK (enter_callbackSYPW),
		      entrySYPW);
    tmp_pos = GTK_ENTRY (entrySYPW)->text_length;
    g_ascii_dtostr(tpoint,24,dec2current(SYPW_DEFAULT_VALUE));
    gtk_editable_insert_text (GTK_EDITABLE (entrySYPW), tpoint , -1, &tmp_pos);
    gtk_editable_select_region (GTK_EDITABLE (entrySYPW),
			        0, GTK_ENTRY (entrySYPW)->text_length);
    gtk_box_pack_start (GTK_BOX (box5), entrySYPW, TRUE, TRUE, 0);
    gtk_widget_show (entrySYPW);

    adjSYPW = gtk_adjustment_new (SYPW_DEFAULT_VALUE, 0,16777215, 10, 1000, 0);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adjSYPW));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box5), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adjSYPW), "value_changed",
                      G_CALLBACK (cb_digits_SynPxlW), NULL);
    
    separator = gtk_hseparator_new ();
    gtk_box_pack_start (GTK_BOX (box5), separator, FALSE, TRUE, 10);
    gtk_widget_show (separator);
    
    label = gtk_label_new ("BIAS SYN :");
    gtk_box_pack_start (GTK_BOX (box5), label, FALSE, FALSE, 0);
    gtk_widget_show (label);

    label = gtk_label_new ("SynThr");
    gtk_box_pack_start (GTK_BOX (box5), label, FALSE, FALSE, 0);
    gtk_widget_show (label);

    entrySYTH = gtk_entry_new ();
    gtk_entry_set_max_length (GTK_ENTRY (entrySYTH), 50);
    g_signal_connect (entrySYTH, "activate",
		      G_CALLBACK (enter_callbackSYTH),
		      entrySYTH);
    tmp_pos = GTK_ENTRY (entrySYTH)->text_length;
    g_ascii_dtostr(tpoint,24,dec2current(SYTH_DEFAULT_VALUE));
    gtk_editable_insert_text (GTK_EDITABLE (entrySYTH), tpoint , -1, &tmp_pos);
    gtk_editable_select_region (GTK_EDITABLE (entrySYTH),
			        0, GTK_ENTRY (entrySYTH)->text_length);
    gtk_box_pack_start (GTK_BOX (box5), entrySYTH, TRUE, TRUE, 0);
    gtk_widget_show (entrySYTH);
    
    adjSYTH = gtk_adjustment_new (SYTH_DEFAULT_VALUE, 0,16777215,10, 1000, 0);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adjSYTH));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box5), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adjSYTH), "value_changed",
                      G_CALLBACK (cb_digits_SynThr), NULL);

    label = gtk_label_new ("SynTau");
    gtk_box_pack_start (GTK_BOX (box5), label, FALSE, FALSE, 0);
    gtk_widget_show (label);

    entrySYTA = gtk_entry_new ();
    gtk_entry_set_max_length (GTK_ENTRY (entrySYTA), 50);
    g_signal_connect (entrySYTA, "activate",
		      G_CALLBACK (enter_callbackSYTA),
		      entrySYTA);
    tmp_pos = GTK_ENTRY (entrySYTA)->text_length;
    g_ascii_dtostr(tpoint,24,dec2current(SYTA_DEFAULT_VALUE));
    gtk_editable_insert_text (GTK_EDITABLE (entrySYTA), tpoint , -1, &tmp_pos);
    gtk_editable_select_region (GTK_EDITABLE (entrySYTA),
			        0, GTK_ENTRY (entrySYTA)->text_length);
    gtk_box_pack_start (GTK_BOX (box5), entrySYTA, TRUE, TRUE, 0);
    gtk_widget_show (entrySYTA);

    adjSYTA = gtk_adjustment_new (SYTA_DEFAULT_VALUE, 0,16777215, 10, 1000, 0);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adjSYTA));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box5), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adjSYTA), "value_changed",
                      G_CALLBACK (cb_digits_SynTau), NULL);

    label = gtk_label_new ("SynW");
    gtk_box_pack_start (GTK_BOX (box5), label, FALSE, FALSE, 0);
    gtk_widget_show (label);

    entrySYW = gtk_entry_new ();
    gtk_entry_set_max_length (GTK_ENTRY (entrySYW), 50);
    g_signal_connect (entrySYW, "activate",
		      G_CALLBACK (enter_callbackSYW),
		      entrySYW);
    tmp_pos = GTK_ENTRY (entrySYW)->text_length;
    g_ascii_dtostr(tpoint,24,dec2current(SYW_DEFAULT_VALUE));
    gtk_editable_insert_text (GTK_EDITABLE (entrySYW), tpoint , -1, &tmp_pos);
    gtk_editable_select_region (GTK_EDITABLE (entrySYW),
			        0, GTK_ENTRY (entrySYW)->text_length);
    gtk_box_pack_start (GTK_BOX (box5), entrySYW, TRUE, TRUE, 0);
    gtk_widget_show (entrySYW);
    
    adjSYW = gtk_adjustment_new (SYW_DEFAULT_VALUE, 0,16777215, 10, 1000, 0);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adjSYW));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box5), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adjSYW), "value_changed",
                      G_CALLBACK (cb_digits_SynW), NULL);
    

    gtk_box_pack_start (GTK_BOX (box3), box5, FALSE, FALSE, 0);
    gtk_widget_show (box5);

    separator = gtk_vseparator_new ();
    gtk_box_pack_start (GTK_BOX (box3), separator, FALSE, TRUE, 10);
    gtk_widget_show (separator);

    //----------BOX7 SUBSECTION:3 of BOX3
    box7 = gtk_vbox_new (FALSE, 0);
    gtk_container_set_border_width (GTK_CONTAINER (box7), 0);

    label = gtk_label_new ("BIAS EM:");
    gtk_box_pack_start (GTK_BOX (box7), label, FALSE, FALSE, 0);
    gtk_widget_show (label);

    label = gtk_label_new ("EMCompH");
    gtk_box_pack_start (GTK_BOX (box7), label, FALSE, FALSE, 0);
    gtk_widget_show (label);

    entryEMCH = gtk_entry_new ();
    gtk_entry_set_max_length (GTK_ENTRY (entryEMCH), 50);
    g_signal_connect (entryEMCH, "activate",
		      G_CALLBACK (enter_callbackEMCH),
		      entryEMCH);
    tmp_pos = GTK_ENTRY (entryEMCH)->text_length;
    g_ascii_dtostr(tpoint,24,dec2current(EMCH_DEFAULT_VALUE));
    gtk_editable_insert_text (GTK_EDITABLE (entryEMCH), tpoint , -1, &tmp_pos);
    gtk_editable_select_region (GTK_EDITABLE (entryEMCH),
			        0, GTK_ENTRY (entryEMCH)->text_length);
    gtk_box_pack_start (GTK_BOX (box7), entryEMCH, TRUE, TRUE, 0);
    gtk_widget_show (entryEMCH);
    
    adjEMCH = gtk_adjustment_new (EMCH_DEFAULT_VALUE, 0,16777215, 10, 1000, 0);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adjEMCH));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box7), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adjEMCH), "value_changed",
                      G_CALLBACK (cb_digits_EMCompH), NULL);

    label = gtk_label_new ("EMCompT");
    gtk_box_pack_start (GTK_BOX (box7), label, FALSE, FALSE, 0);
    gtk_widget_show (label);

    entryEMCT = gtk_entry_new ();
    gtk_entry_set_max_length (GTK_ENTRY (entryEMCT), 50);
    g_signal_connect (entryEMCT, "activate",
		      G_CALLBACK (enter_callbackEMCT),
		      entryEMCT);
    tmp_pos = GTK_ENTRY (entryEMCT)->text_length;
    g_ascii_dtostr(tpoint,24,dec2current(EMCT_DEFAULT_VALUE));
    gtk_editable_insert_text (GTK_EDITABLE (entryEMCT), tpoint , -1, &tmp_pos);
    gtk_editable_select_region (GTK_EDITABLE (entryEMCT),
			        0, GTK_ENTRY (entryEMCT)->text_length);
    gtk_box_pack_start (GTK_BOX (box7), entryEMCT, TRUE, TRUE, 0);
    gtk_widget_show (entryEMCT);
    
    adjEMCT = gtk_adjustment_new (EMCT_DEFAULT_VALUE, 0,16777215, 10, 1000, 0);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adjEMCT));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box7), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adjEMCT), "value_changed",
                      G_CALLBACK (cb_digits_EMCompT), NULL);

    label = gtk_label_new ("EMVrefL");
    gtk_box_pack_start (GTK_BOX (box7), label, FALSE, FALSE, 0);
    gtk_widget_show (label);

    entryEMVL = gtk_entry_new ();
    gtk_entry_set_max_length (GTK_ENTRY (entryEMVL), 50);
    g_signal_connect (entryEMVL, "activate",
		      G_CALLBACK (enter_callbackEMVL),
		      entryEMVL);
    tmp_pos = GTK_ENTRY (entryEMVL)->text_length;
    g_ascii_dtostr(tpoint,24,dec2current(EMVL_DEFAULT_VALUE));
    gtk_editable_insert_text (GTK_EDITABLE (entryEMVL), tpoint , -1, &tmp_pos);
    gtk_editable_select_region (GTK_EDITABLE (entryEMVL),
			        0, GTK_ENTRY (entryEMVL)->text_length);
    gtk_box_pack_start (GTK_BOX (box7), entryEMVL, TRUE, TRUE, 0);
    gtk_widget_show (entryEMVL);
    
    adjEMVL = gtk_adjustment_new (EMVL_DEFAULT_VALUE, 0,16777215, 10, 1000, 0);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adjEMVL));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box7), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adjEMVL), "value_changed",
                      G_CALLBACK (cb_digits_EMVrefL), NULL); 

    label = gtk_label_new ("EMVrefH");
    gtk_box_pack_start (GTK_BOX (box7), label, FALSE, FALSE, 0);
    gtk_widget_show (label);

    entryEMVH = gtk_entry_new ();
    gtk_entry_set_max_length (GTK_ENTRY (entryEMVH), 50);
    g_signal_connect (entryEMVH, "activate",
		      G_CALLBACK (enter_callbackEMVH),
		      entryEMVH);
    tmp_pos = GTK_ENTRY (entryEMVH)->text_length;
    g_ascii_dtostr(tpoint,24,dec2current(EMVH_DEFAULT_VALUE));
    gtk_editable_insert_text (GTK_EDITABLE (entryEMVH), tpoint , -1, &tmp_pos);
    gtk_editable_select_region (GTK_EDITABLE (entryEMVH),
			        0, GTK_ENTRY (entryEMVH)->text_length);
    gtk_box_pack_start (GTK_BOX (box7), entryEMVH, TRUE, TRUE, 0);
    gtk_widget_show (entryEMVH);
    
    adjEMVH = gtk_adjustment_new (EMVH_DEFAULT_VALUE, 0,16777215, 10, 1000, 0);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adjEMVH));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box7), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adjEMVH), "value_changed",
                      G_CALLBACK (cb_digits_EMVrefH), NULL);
    
    separator = gtk_hseparator_new ();
    gtk_box_pack_start (GTK_BOX (box7), separator, FALSE, TRUE, 10);
    gtk_widget_show (separator);

    label = gtk_label_new ("BIAS EXTRA :");
    gtk_box_pack_start (GTK_BOX (box7), label, FALSE, FALSE, 0);
    gtk_widget_show (label);

    label = gtk_label_new ("Foll");
    gtk_box_pack_start (GTK_BOX (box7), label, FALSE, FALSE, 0);
    gtk_widget_show (label);

    entryFOLL = gtk_entry_new ();
    gtk_entry_set_max_length (GTK_ENTRY (entryFOLL), 50);
    g_signal_connect (entryFOLL, "activate",
		      G_CALLBACK (enter_callbackFOLL),
		      entryFOLL);
    tmp_pos = GTK_ENTRY (entryFOLL)->text_length;
    g_ascii_dtostr(tpoint,24,dec2current(FOLL_DEFAULT_VALUE));
    gtk_editable_insert_text (GTK_EDITABLE (entryFOLL), tpoint , -1, &tmp_pos);
    gtk_editable_select_region (GTK_EDITABLE (entryFOLL),
			        0, GTK_ENTRY (entryFOLL)->text_length);
    gtk_box_pack_start (GTK_BOX (box7), entryFOLL, TRUE, TRUE, 0);
    gtk_widget_show (entryFOLL);
    
    adjFOLL = gtk_adjustment_new (FOLL_DEFAULT_VALUE, 0,16777215, 10, 1000, 0);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adjFOLL));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box7), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adjFOLL), "value_changed",
                      G_CALLBACK (cb_digits_Foll), NULL);

    label = gtk_label_new ("ReqPuX");
    gtk_box_pack_start (GTK_BOX (box7), label, FALSE, FALSE, 0);
    gtk_widget_show (label);

    entryRPX = gtk_entry_new ();
    gtk_entry_set_max_length (GTK_ENTRY (entryRPX), 50);
    g_signal_connect (entryRPX, "activate",
		      G_CALLBACK (enter_callbackRPX),
		      entryRPX);
    tmp_pos = GTK_ENTRY (entryRPX)->text_length;
    g_ascii_dtostr(tpoint,24,dec2current(RPX_DEFAULT_VALUE));
    gtk_editable_insert_text (GTK_EDITABLE (entryRPX), tpoint , -1, &tmp_pos);
    gtk_editable_select_region (GTK_EDITABLE (entryRPX),
			        0, GTK_ENTRY (entryRPX)->text_length);
    gtk_box_pack_start (GTK_BOX (box7), entryRPX, TRUE, TRUE, 0);
    gtk_widget_show (entryRPX);
    
    adjRPX = gtk_adjustment_new (RPX_DEFAULT_VALUE, 0,16777215, 10 ,1000, 0);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adjRPX));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box7), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adjRPX), "value_changed",
                      G_CALLBACK (cb_digits_ReqPuX), NULL);

    label = gtk_label_new ("ReqPuY");
    gtk_box_pack_start (GTK_BOX (box7), label, FALSE, FALSE, 0);
    gtk_widget_show (label);

    entryRPY = gtk_entry_new ();
    gtk_entry_set_max_length (GTK_ENTRY (entryRPY), 50);
    g_signal_connect (entryRPY, "activate",
		      G_CALLBACK (enter_callbackRPY),
		      entryRPY);
    tmp_pos = GTK_ENTRY (entryRPY)->text_length;
    g_ascii_dtostr(tpoint,24,dec2current(RPY_DEFAULT_VALUE));
    gtk_editable_insert_text (GTK_EDITABLE (entryRPY), tpoint , -1, &tmp_pos);
    gtk_editable_select_region (GTK_EDITABLE (entryRPY),
			        0, GTK_ENTRY (entryRPY)->text_length);
    gtk_box_pack_start (GTK_BOX (box7), entryRPY, TRUE, TRUE, 0);
    gtk_widget_show (entryRPY);
    
    adjRPY = gtk_adjustment_new (RPY_DEFAULT_VALUE, 0,16777215, 10, 1000, 0);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adjRPY));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box7), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adjRPY), "value_changed",
                      G_CALLBACK (cb_digits_ReqPuY), NULL);
    
    label = gtk_label_new ("ArbPd");
    gtk_box_pack_start (GTK_BOX (box7), label, FALSE, FALSE, 0);
    gtk_widget_show (label);

    entryARBP = gtk_entry_new ();
    gtk_entry_set_max_length (GTK_ENTRY (entryARBP), 50);
    g_signal_connect (entryARBP, "activate",
		      G_CALLBACK (enter_callbackARBP),
		      entryARBP);
    tmp_pos = GTK_ENTRY (entryARBP)->text_length;
    g_ascii_dtostr(tpoint,24,dec2current(ARBP_DEFAULT_VALUE));
    gtk_editable_insert_text (GTK_EDITABLE (entryARBP), tpoint , -1, &tmp_pos);
    gtk_editable_select_region (GTK_EDITABLE (entryARBP),
			        0, GTK_ENTRY (entryARBP)->text_length);
    gtk_box_pack_start (GTK_BOX (box7), entryARBP, TRUE, TRUE, 0);
    gtk_widget_show (entryARBP);
    
    adjARBP = gtk_adjustment_new (ARBP_DEFAULT_VALUE, 0,16777215, 10, 1000, 0);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adjARBP));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box7), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adjARBP), "value_changed",
                      G_CALLBACK (cb_digits_ArbPd), NULL);

    label = gtk_label_new ("testPBias");
    gtk_box_pack_start (GTK_BOX (box7), label, FALSE, FALSE, 0);
    gtk_widget_show (label);
    
    entryTPB = gtk_entry_new ();
    gtk_entry_set_max_length (GTK_ENTRY (entryTPB), 50);
    g_signal_connect (entryTPB, "activate",
		      G_CALLBACK (enter_callbackTPB),
		      entryTPB);
    tmp_pos = GTK_ENTRY (entryTPB)->text_length;
    g_ascii_dtostr(tpoint,24,dec2current(TPB_DEFAULT_VALUE));
    gtk_editable_insert_text (GTK_EDITABLE (entryTPB), tpoint , -1, &tmp_pos);
    gtk_editable_select_region (GTK_EDITABLE (entryTPB),
			        0, GTK_ENTRY (entryTPB)->text_length);
    gtk_box_pack_start (GTK_BOX (box7), entryTPB, TRUE, TRUE, 0);
    gtk_widget_show (entryTPB);
    
    adjTPB = gtk_adjustment_new (TPB_DEFAULT_VALUE, 0,16777215, 10, 1000, 0);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adjTPB));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box7), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adjTPB), "value_changed",
                      G_CALLBACK (cb_digits_testPbias), NULL);
    
    
    gtk_box_pack_start (GTK_BOX (box3), box7, FALSE, FALSE, 0);
    gtk_widget_show (box7);

    separator = gtk_vseparator_new ();
    gtk_box_pack_start (GTK_BOX (box3), separator, FALSE, TRUE, 10);
    gtk_widget_show (separator);


    //----------BOX6 SUBSECTION:4 of BOX3
    box6 = gtk_vbox_new (FALSE, 0);
    gtk_container_set_border_width (GTK_CONTAINER (box6), 0);

    label = gtk_label_new ("FUNCTIONS:");
    gtk_box_pack_start (GTK_BOX (box6), label, FALSE, FALSE, 0);
    gtk_widget_show (label);

    
    buttonProgBias = gtk_button_new ();
    g_signal_connect (G_OBJECT (buttonProgBias), "clicked", G_CALLBACK (callbackProgBiasButton),(gpointer) "left");
    boxButton = xpm_label_box (NULL, (gchar*) "ProgBias Left");
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (buttonProgBias), boxButton);
    gtk_container_add (GTK_CONTAINER (buttonProgBias), box6);
    gtk_widget_show (buttonProgBias);
    gtk_box_pack_start (GTK_BOX (box6), buttonProgBias, FALSE, FALSE, 10);
    gtk_widget_show (buttonProgBias);
    
    
    
    buttonProgBias = gtk_button_new ();
    g_signal_connect (G_OBJECT (buttonProgBias), "clicked", G_CALLBACK (callbackProgBiasButton),(gpointer) "right");
    boxButton = xpm_label_box (NULL,  (gchar*)"ProgBias Right");
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (buttonProgBias), boxButton);
    gtk_container_add (GTK_CONTAINER (buttonProgBias), box6);
    gtk_widget_show (buttonProgBias);
    gtk_box_pack_start (GTK_BOX (box6), buttonProgBias, FALSE, FALSE, 10);
    gtk_widget_show (buttonProgBias);
    
    
    
    buttonSave = gtk_button_new ();
    g_signal_connect (G_OBJECT (buttonSave), "clicked", G_CALLBACK (callbackSaveButton), NULL);
    boxButton = xpm_label_box (NULL,  (gchar*)"Save");
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (buttonSave), boxButton);
    gtk_container_add (GTK_CONTAINER (buttonSave), box6);
    gtk_box_pack_start (GTK_BOX (box6), buttonSave, FALSE, FALSE, 10);
    gtk_widget_show (buttonSave);

    buttonLoad = gtk_button_new ();
    g_signal_connect (G_OBJECT (buttonLoad), "clicked", G_CALLBACK (callbackLoadButton), NULL);
    boxButton = xpm_label_box (NULL,  (gchar*)"Load");
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (buttonLoad), boxButton);
    gtk_container_add (GTK_CONTAINER (buttonLoad), box6);
    gtk_box_pack_start (GTK_BOX (box6), buttonLoad, FALSE, FALSE, 10);
    gtk_widget_show (buttonLoad);  

    entrySAVELOAD = gtk_entry_new ();
    gtk_entry_set_max_length (GTK_ENTRY (entrySAVELOAD), 50);
    gtk_entry_set_text (GTK_ENTRY (entrySAVELOAD), "filename");
    g_signal_connect (entrySAVELOAD, "activate",
		      G_CALLBACK (enter_callbackSAVELOAD),
		      entrySAVELOAD);
    tmp_pos = GTK_ENTRY (entrySAVELOAD)->text_length;
    gtk_editable_insert_text (GTK_EDITABLE (entrySAVELOAD), ".txt" , -1, &tmp_pos);
    gtk_editable_select_region (GTK_EDITABLE (entrySAVELOAD),
			        0, GTK_ENTRY (entrySAVELOAD)->text_length);
    gtk_box_pack_start (GTK_BOX (box6), entrySAVELOAD, TRUE, TRUE, 0);
    gtk_widget_show (entrySAVELOAD);
    
    buttonDumpOn = gtk_button_new ();
    g_signal_connect (G_OBJECT (buttonDumpOn), "clicked", G_CALLBACK (callbackDumpButton),(gpointer) "on");
    boxButton = xpm_label_box (NULL,  (gchar*)"DumpOn");
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (buttonDumpOn), boxButton);
    gtk_container_add (GTK_CONTAINER (buttonDumpOn), box6);
    gtk_box_pack_start (GTK_BOX (box6), buttonDumpOn, FALSE, FALSE, 10);
    gtk_widget_show (buttonDumpOn);  
    
    buttonDumpOff = gtk_button_new ();
    g_signal_connect (G_OBJECT (buttonDumpOff), "clicked", G_CALLBACK (callbackDumpButton),(gpointer) "off");
    boxButton = xpm_label_box (NULL,  (gchar*)"DumpOff");
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (buttonDumpOff), boxButton);
    gtk_container_add (GTK_CONTAINER (buttonDumpOff), box6);
    gtk_box_pack_start (GTK_BOX (box6), buttonDumpOff, FALSE, FALSE, 10);
    gtk_widget_show (buttonDumpOff);  

    gtk_box_pack_start (GTK_BOX (box3), box6, FALSE, FALSE, 0);
    gtk_widget_show (box6);

    /*
    label = gtk_label_new ("Processing Options:");
    gtk_box_pack_start (GTK_BOX (box3), label, FALSE, FALSE, 0);
    gtk_widget_show (label);
    */


    //scrollbar = gtk_hscrollbar_new (GTK_ADJUSTMENT (adj1));
    // Notice how this causes the scales to always be updated
    // continuously when the scrollbar is moved 
    //gtk_range_set_update_policy (GTK_RANGE (scrollbar), 
    //                             GTK_UPDATE_CONTINUOUS);
    //gtk_box_pack_start (GTK_BOX (box3), scrollbar, TRUE, TRUE, 0);
    //gtk_widget_show (scrollbar;)

    //-----Check Buttons
    box4=  gtk_vbox_new (FALSE, 0);
    
    gtk_box_pack_start (GTK_BOX (box3), box4, TRUE, TRUE, 0);
    gtk_widget_show (box4);

    //-----box4
    box4=  gtk_vbox_new (FALSE, 0);
    gtk_box_pack_start (GTK_BOX (box3), box4, TRUE, TRUE, 0);
    gtk_widget_show (box4);
    //---box 4


    separator = gtk_hseparator_new ();
    gtk_box_pack_start (GTK_BOX (box2), separator, FALSE, TRUE, 10);
    gtk_widget_show (separator);

    /*
    label = gtk_label_new ("coefficient motion");
    gtk_box_pack_start (GTK_BOX (box2), label, FALSE, FALSE, 0);
    gtk_widget_show (label);
    
    adjMotion = gtk_adjustment_new (0.5, 0.0,1.0,0.01, 0.0, 0.0);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adjMotion));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box2), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adjMotion), "value_changed",
                      G_CALLBACK (cb_digits_scaleMotion), NULL);
                      */
    
    


    //gtk_container_add (GTK_CONTAINER (box2), boxSliders);
    gtk_box_pack_start(GTK_BOX(box), box2,FALSE,FALSE, 10);
    // StatusBar for main window
    //statusbar = gtk_statusbar_new ();
    //updateStatusbar(GTK_STATUSBAR (statusbar));
    //gtk_box_pack_start (GTK_BOX (box), statusbar, FALSE, TRUE, 0);
    //gtk_widget_size_request(statusbar, &actualSize);
    //_occupiedHeight += 2*(actualSize.height);*/

    frame = gdk_pixbuf_new (GDK_COLORSPACE_RGB, FALSE, 8, 320, 240);
    // TimeOut used to refresh the screen
    timeout_ID = gtk_timeout_add (100,timeout_CB, NULL);

    mainWindow=window;

    return window;
}

void configure(yarp::os::ResourceFinder rf){
    /* Process all parameters from both command-line and .ini file */
    /* get the module name which will form the stem of all module port names */
    _options.portName      = rf.check("name", 
                           Value("/asvBiasInterface"), 
                           "module name (string)").asString();
    _options.posX      = rf.check("x", 
                           Value(100), 
                           "module pos x (int)").asInt();
    _options.posY      = rf.check("y", 
                           Value(100), 
                           "module pos y (int)").asInt();

}

void setOptionsToDefault() {
    // Options defaults
    _options.refreshTime = 100;
    _options.outputEnabled = 0;
    _options.windWidth = 300;
    _options.windHeight = 300;
    _options.posX = 100;
    _options.posY = 100;
    _options.saveOnExit = 0;
}

bool openPorts() {
    //_pOutPort = new yarp::os::BufferedPort<yarp::os::Bottle>;
    _pOutPort=new Port;
    _options.portName+="/command:o";
    bool ok = _pOutPort->open(_options.portName.c_str());
    if (ok) {
        g_print("Port registration succeed!\n");
    }
    else {
            g_print("ERROR: Port registration failed.\nQuitting, sorry.\n");
            return false;
    }
    return true;
}

void closePorts() {
    _pOutPort->close();
    bool ok = true;
    if  (ok)
        printf("Port unregistration succeed!\n");
    else 
        printf("ERROR: Port unregistration failed.\n");
    delete _pOutPort;
    _pOutPort = NULL;

}

void cleanExit() {
    if (timeout_ID!=0)
        g_source_remove (timeout_ID);
    timeout_ID = 0;
    
    g_source_remove(timeout_update_ID);
    timeout_update_ID=0;

    gtk_main_quit ();
}

//-------------------------------------------------
// Main
//-------------------------------------------------
#undef main //ace leaves a "main" macro defined

int myMain(int argc, char* argv[]) {
    yarp::os::Network yarp;

    
    //initialize threads in gtk, copied almost verbatim from
    // http://library.gnome.org/devel/gdk/unstable/gdk-Threads.htm
    //g_thread_init (NULL);
    //gdk_threads_init ();
    //gdk_threads_enter ();
    createObjects();
    _frameN = 0;
    timeout_ID = 0;
    setOptionsToDefault();

    yarp::os::ResourceFinder* rf;
    rf=new ResourceFinder();
    rf->setVerbose(true);
    rf->setDefaultConfigFile("asvBiasInterface.ini"); //overridden by --from parameter
    rf->setDefaultContext("eMorphApp/conf");   //overridden by --context parameter
    rf->configure("ICUB_ROOT", argc, argv);
    configure(*rf);
    
    // Parse command line parameters, do this before
    // calling gtk_init(argc, argv) otherwise weird things 
    // happens
    if (!openPorts())
        goto exitRoutine;

    // This is called in all GTK applications. Arguments are parsed
    // from the command line and are returned to the application.
    gtk_init (&argc, &argv);

    // create a new window
    mainWindow = createMainWindow();
    
    // Non Modal Dialogs
#if GTK_CHECK_VERSION(2,6,0)
    saveSingleDialog = createSaveSingleDialog();
    saveSetDialog = createSaveSetDialog();
#else
    printf("Functionality omitted for older GTK version\n");
#endif
    // Shows all widgets in main Window
    gtk_widget_show_all (mainWindow);
    gtk_window_move(GTK_WINDOW(mainWindow), _options.posX, _options.posY);
    // All GTK applications must have a gtk_main(). Control ends here
    // and waits for an event to occur (like a key press or
    // mouse event).

    //ptr_portCallback->attach(&_resources);
    //ptr_portCallback->attach(&portFpsData);
    //ptr_inputPort->useCallback(*ptr_portCallback);

    if (_options.synch)
    {
        setSynchroMode();
        gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(synchroDisplayItem), true);
    }
    else
    {
        setTimedMode(_options.refreshTime);
        gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(synchroDisplayItem), false);
    }
    gtk_main ();

exitRoutine:
    // leave critical section here. From example
    // http://library.gnome.org/devel/gdk/unstable/gdk-Threads.htm
    //gdk_threads_leave ();

    closePorts();
    deleteObjects();
    return 0;
}

#ifdef YARP_WIN32_NOCONSOLE
#include <windows.h>
// win32 non-console applications define WinMain as the
// entry point for the linker
int WINAPI WinMain(HINSTANCE hInstance,
                   HINSTANCE hPrevInstance,
                   LPSTR lpCmdLine,
                   int nCmdShow)
{
    return myMain (__argc, __argv);
}
#else
int main(int argc, char* argv[]) {
    return myMain(argc, argv);
}
#endif

void printHelp() {
    g_print("asvBiasInterface usage:\n");
    g_print("--name: input port name (default: /selAttentionInterface)\n");
}


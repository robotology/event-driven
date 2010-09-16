/*
 * USB retina driver
 * Martin Ebner, IGI / TU Graz (ebner at igi.tugraz.at)
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, version 2.
 *
 * This driver is based on the 2.6.3 version of drivers/usb/usb-skeleton.c
 * (Copyright (C) 2001-2004 Greg Kroah-Hartman (greg@kroah.com))
 *
 */

#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>

#define DEVICE_FILE_NAME "/dev/retina0"


int main(int argc, char *argv[])
{
	int file_desc,len=0,i;
	char *buffer=(char *)malloc(512);
	printf("retina_test.c\n");
	/* open device file /dev/retina0*/
	file_desc = open(DEVICE_FILE_NAME, O_RDWR);
	if (file_desc < 0) {
		printf("Can't open device file: %s\n", DEVICE_FILE_NAME);
		exit(-1);
	}
	/* read address events from device file */
	for (i=0;i<1000;i++)
	{
	    printf("Reading ...");
            len += read(file_desc,buffer,512);
	}	
	printf("%d address events read\n",len/4);
	close(file_desc);
	return 0;
}

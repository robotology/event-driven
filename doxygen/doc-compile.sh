#!/bin/bash

SOURCE=../
DESTINATION=generated-from-xml
export YARP_ROOT=/home/aglover/projects/yarp

# clean-up
rm doc -rf
rm $DESTINATION -rf

# generate doxy from xml
mkdir $DESTINATION
list=`find $SOURCE -iname *.xml | xargs`
for i in $list
do
   filename=`basename $i`
   doxyfile=${filename%%.*}
   xsltproc --output $DESTINATION/$doxyfile.dox $YARP_ROOT/scripts/yarp-module.xsl $i
done

doxygen ./generate.txt

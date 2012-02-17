#!/usr/bin/env python
#
# Controller for IcubInterface
#
# (c) 2011 Francesco Rea @ Italian Institute of Techology

#!/usr/bin/python

import yarp
import optparse

def main():
	print 'YARP Python bindings problem'

	usage = "usage: %prog [options]"
	parser = optparse.OptionParser(usage)

	parser.add_option('-p', '--port', dest='port', default='/py/datagen:o', 
                      help='name of the output port')
	parser.add_option('-i', '--input-port', dest='inputport', 
                      default='/py/reader:i', help='name of the input port')
	parser.add_option('-t', '--type', dest='type', default=1, type='int',
                      help='1: local callback, 2: separate callback class')
	parser.add_option('--robot', dest='robot', default='iCub', 
                      help='iCub: robot, motor: singleMotor')
	parser.add_option('--part', dest='part', default='head', 
                      help='iCub: robot, motor: singleMotor')

	(opt, args) = parser.parse_args()

	print opt, args

	#initilisation
	yarp.Network.init()
	p = yarp.BufferedPortBottle()
	p.open("/singleMotorControl");

	#trying to active the following classes
	#ia = yarp.IAmplifierControl()
	#ip = yarp.IPidControl()
	#ic = yarp.IControlMode()
	
	#finding the correct properties
	options = yarp.Property()
	options.put("device", "remote_controlboard");
	nameLocal  = "/iCub/head/client"
	nameRemote = "/iCub/head"
	options.put("local" , nameLocal);
	options.put("remote", nameRemote);

	# activating the PolyDriver
	poly = yarp.PolyDriver()
	if poly.isValid():
		print "Poly valid"
		# activating other component s
		ip = poly.viewIPidControl()		
		ia = poly.viewIAmplifierControl()	
		ic = poly.viewIControlMode()
		# active part 
		#poly.viewIAmplifierControl().enableAmp(1)
		ia.enableAmp(1)		
	else:
		print "Poly not valid"
	bottle = p.prepare()
	bottle.clear()
	bottle.addString("created PolyDriver")	
	p.write()


	#closing the script with a countDown
	top = 10;
	for i in range(1,top):
		bottle = p.prepare()
		bottle.clear()
		bottle.addString("count")
		bottle.addInt(top - i)
		bottle.addString("seconds")
		print "Time to end", bottle.toString()
		p.write()
		yarp.Time.delay(1.0)

	p.close();

	yarp.Network.fini();

if __name__ == "__main__":
    main()
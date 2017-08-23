#!/usr/bin/env python
'''
Example Module
Peter barker, September 2016

This module simply serves as a starting point for your own MAVProxy module.

1. copy this module sidewise (e.g. "cp mavproxy_example.py mavproxy_coolfeature.py"
2. replace all instances of "example" with whatever your module should be called
(e.g. "coolfeature")

3. trim (or comment) out any functionality you do not need
'''

import os
import os.path
import sys
from pymavlink import mavutil
import errno
import time
import gphoto2 as gp
import subprocess
import time
import datetime
#import mavproxy.sc_ExifWriter as exifWriter

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings


class llcamera(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module"""
        super(llcamera, self).__init__(mpstate, "example", "")
        self.status_callcount = 0
        self.boredom_interval = 10 # seconds
        self.last_bored = time.time()

        self.packets_mytarget = 0
        self.packets_othertarget = 0
        self.verbose = False

        self.example_settings = mp_settings.MPSettings(
            [ ('verbose', bool, False),
          ])
        self.add_command('llcamera', self.cmd_example, "llcamera module", ['status','set (LOGSETTING)'])
	self.context = gp.gp_context_new()
	self.camera = gp.check_result(gp.gp_camera_new())
	self.path = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M")
	self.targetPath = os.path.join('/home/ubuntu/Pictures', self.path)
	self.vehicleLat = 0.0              # Current Vehicle Latitude
        self.vehicleLon = 0.0              # Current Vehicle Longitude
        self.vehicleHdg = 0.0              # Current Vehicle Heading
        self.vehicleAMSL = 0.0             # Current Vehicle Altitude above mean sea level

        self.vehicleRoll = 0.0              # Current Vehicle Roll
        self.vehiclePitch = 0.0              # Current Vehicle Pitch
	self.vehicleYaw = 0.0
	self.imageLogFilePath = os.path.join(self.targetPath, 'log_' + self.path + '.txt')
	print(self.imageLogFilePath)
	if not os.path.exists(self.targetPath):
    		os.makedirs(self.targetPath)
		file = open(self.imageLogFilePath, 'w') 
		file.write('Filename\tLatitude\tLongitude\tAlt (AMSL)\tRoll\tPitch\tYaw\n')
		file.close()
	while True:
		try:
			self.camera.init(self.context)
		except gp.GPhoto2Error as ex:
			if ex.code == gp.GP_ERROR_MODEL_NOT_FOUND:
		    # no camera, try again in 2 seconds
				time.sleep(2)
				continue
		# some other error we can't handle here
			raise
	    # operation completed successfully so exit loop
		break
	# continue with rest of program
	print('Camera connected.')

    def usage(self):
        '''show help on command line options'''
        return "Usage: llcamera <status|set>"

    def cmd_example(self, args):
        '''control behaviour of the module'''
        if len(args) == 0:
            print self.usage()
        elif args[0] == "status":
            print self.status()
        elif args[0] == "set":
            self.example_settings.command(args[1:])
        else:
            print self.usage()

    def status(self):
        '''returns information about module'''
        self.status_callcount += 1
        self.last_bored = time.time() # status entertains us

        return("status called %(status_callcount)d times.  My target positions=%(packets_mytarget)u  Other target positions=%(packets_mytarget)u" %
               {"status_callcount": self.status_callcount,
                "packets_mytarget": self.packets_mytarget,
                "packets_othertarget": self.packets_othertarget,
               })

    def boredom_message(self):
        if self.example_settings.verbose:
            return ("I'm very bored")
        return ("I'm bored")

    def idle_task(self):
        '''called rapidly by mavproxy'''
        now = time.time()
        if now-self.last_bored > self.boredom_interval:
            self.last_bored = now
            message = self.boredom_message()
            #self.say("%s: %s" % (self.name,message))
            # See if whatever we're connected to would like to play:
            self.master.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_NOTICE,
                                            message)

    def mavlink_packet(self, m):
        '''handle mavlink packets'''
        if m.get_type() == 'GLOBAL_POSITION_INT':
            if self.settings.target_system == 0 or self.settings.target_system == m.get_srcSystem():
                self.packets_mytarget += 1
            else:
                self.packets_othertarget += 1
	    (self.vehicleLat, self.vehicleLon, self.vehicleHdg, self.vehicleAMSL) = (m.lat*1.0e-7, m.lon*1.0e-7, m.hdg*0.01, m.alt*0.001)
	#if mtype == "ATTITUDE":
	    (self.vehicleRoll, self.vehiclePitch, self.vehicleYaw) = (math.degrees(m.roll), math.degrees(m.pitch), math.degrees(m.yaw))
	if m.get_type() == "CAMERA_STATUS":
		print ("Got Message camera_status")
	if m.get_type() == "CAMERA_FEEDBACK":
		print ("Got Message camera_feedback")
		'''self.__vCmdCamTrigger(m)'''	
		file_path = gp.check_result(gp.gp_camera_capture(self.camera, gp.GP_CAPTURE_IMAGE, self.context))

		geotagFile = open(self.imageLogFilePath, 'a')
		params = "%s\t%f\t%f\t%f\t%f\t%f\t%f\n" % (file_path.name, self.vehicleLat, self.vehicleLon, self.vehicleAMSL, self.vehicleRoll, self.vehiclePitch, self.vehicleYaw)
		geotagFile.write(params)
		geotagFile.close()
		print(params)
    		print('Camera file path: {0}/{1}'.format(file_path.folder, file_path.name))
		target = os.path.join(self.targetPath, file_path.name)
		print('Copying image to', target)
		camera_file = gp.check_result(gp.gp_camera_file_get(self.camera, file_path.folder, file_path.name, gp.GP_FILE_TYPE_NORMAL, self.context))
		gp.check_result(gp.gp_file_save(camera_file, target))
		gp.check_result(gp.gp_camera_exit(camera, self.context))
		
	if m.get_type() == "COMMAND_LONG":
		if m.command == mavutil.mavlink.MAV_CMD_DO_DIGICAM_CONFIGURE:
			print ("Got Message Digicam_configure")
		elif m.command == mavutil.mavlink.MAV_CMD_DO_DIGICAM_CONTROL:
			print ("Got Message Digicam_control")

def init(mpstate):
    '''initialise module'''
    return llcamera(mpstate)

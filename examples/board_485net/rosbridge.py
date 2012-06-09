#!/usr/bin/env python

import roslib
roslib.load_manifest('net_485net_simbridge')
import rospy
from packets_485net.msg import *

import sys
import os
import struct
import binascii

if len(sys.argv) < 3:
	print "Usage: %s toros fromros"
	sys.exit(0)

toros = os.fdopen(os.open(sys.argv[1], os.O_RDONLY), "r")
fromros = os.fdopen(os.open(sys.argv[2], os.O_WRONLY), "w", 0)

rospy.init_node('net_485net_simbridge')

def packet_to_sim(pkt):
	print "<-- %s" % binascii.hexlify(pkt.data)
	fromros.write(struct.pack("B", len(pkt.data)))
	fromros.write(pkt.data)

pub = rospy.Publisher("net_485net_incoming_packets", packet_485net_raw)
rospy.Subscriber("net_485net_outgoing_packets", packet_485net_raw, packet_to_sim)

while True:
	l = struct.unpack("B", toros.read(1))[0]
	#print len
	pkt = toros.read(l)
	print "--> %s" % binascii.hexlify(pkt)
	rospkt = packet_485net_raw()
	rospkt.header.stamp = rospy.Time.now()
	rospkt.data = pkt
	pub.publish(rospkt)

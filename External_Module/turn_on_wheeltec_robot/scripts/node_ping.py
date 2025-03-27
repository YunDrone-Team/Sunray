#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import time
import rospy
import rosnode
import rosgraph
import rosgraph.names
import errno
import socket 
from geometry_msgs.msg import Twist, Vector3
try:
    from xmlrpc.client import ServerProxy
except ImportError:
    from xmlrpclib import ServerProxy

try: 
    import urllib.parse as urlparse
except ImportError:
    import urlparse

"""
This sample integrated source code.
If you want a simple way to ping rosnode.
rosnode.rosnode_ping('/node_name', max_count=1)
max_count : Number of pings
"""

class ROSNodeException(Exception):
    """
    rosnode base exception type
    """
    pass
class ROSNodeIOException(ROSNodeException):
    """
    Exceptions for communication-related (i/o) errors, generally due to Master or Node network communication issues.
    """
    pass

class Node_Detecion:
    def __init__(self):
        self.node_name = rospy.get_param('~node_name')
        #self.check_frequency = rospy.get_param('~frequency')
        self.cmdVelPublisher = rospy.Publisher('/cmd_vel', Twist, queue_size =3)
        self._caller_apis = {}
        self.find = None

    def stopMoving(self):
        velocity = Twist()
        velocity.linear = Vector3(0.,0.,0.)
        velocity.angular= Vector3(0.,0.,0.)
        self.cmdVelPublisher.publish(velocity)

    def _succeed(self,args):
        code, msg, val = args
        if code != 1:
            raise ROSNodeException("remote call failed: %s"%msg)
        return val

    def get_api_uri(self, master, caller_id, skip_cache=False):
        """
        @param master: rosgraph Master instance
        @type  master: rosgraph.Master
        @param caller_id: node name
        @type  caller_id: str
        @param skip_cache: flag to skip cached data and force to lookup node from master
        @type  skip_cache: bool
        @return: xmlrpc URI of caller_id
        @rtype: str
        @raise ROSNodeIOException: if unable to communicate with master
        """
        caller_api = self._caller_apis.get(caller_id, None)
        if not caller_api or skip_cache:
            try:
                caller_api = master.lookupNode(caller_id)
                self._caller_apis[caller_id] = caller_api
            except rosgraph.MasterError:
                return None
            except socket.error:
                raise ROSNodeIOException("Unable to communicate with master!")
        return caller_api

    def rosnode_ping(self, node_name, max_count=None, details=False,skip_cache=False):
        """
        Test connectivity to node by calling its XMLRPC API
        @param node_name: name of node to ping
        @type  node_name: str
        @param max_count: number of ping requests to make
        @type  max_count: int
        @param details: print ping information to screen
        @type  details: bool
        @return: True if node pinged
        @rtype: bool
        @raise ROSNodeIOException: if unable to communicate with master
        """
        ID = '/rosnode'
        master = rosgraph.Master(ID)
        node_api = self.get_api_uri(master,node_name)
        if not node_api:
            print("cannot ping [%s] "%node_name)
            return False

        timeout = 3.

        if details:
            print("pinging %s with a timeout of %ss"%(node_name, timeout))
        socket.setdefaulttimeout(timeout)
        node = ServerProxy(node_api)
        lastcall = 0.
        count = 0
        acc = 0.
        try:
            while True:
                try:
                    start = time.time()
                    pid = self._succeed(node.getPid(ID))
                    end = time.time()
                    self.find = 1

                    dur = (end-start)*1000.
                    acc += dur
                    count += 1

                    if details:
                        print("xmlrpc reply from %s\ttime=%fms"%(node_api, dur))
                    # 1s between pings
                except socket.error as e:
                    # 3786: catch ValueError on unpack as socket.error is not always a tuple
                    try:
                        # #3659
                        errnum, msg = e.args
                        if errnum == -2: #name/service unknown
                            p = urlparse.urlparse(node_api)
                            print("ERROR: Unknown host [%s] for node [%s]"%(p.hostname, node_name))
                        elif errnum == errno.ECONNREFUSED:
                            # check if node url has changed
                            new_node_api = self.get_api_uri(master,node_name, skip_cache=True)
                            if not new_node_api:
                                print("cannot ping [%s]: unknown node"%node_name)
                                return False
                            if new_node_api != node_api:
                                if details:
                                    print("node url has changed from [%s] to [%s], retrying to ping"%(node_api, new_node_api))
                                node_api = new_node_api
                                node = ServerProxy(node_api)
                                continue
                            print("ERROR: connection refused to [%s]"%(node_api))
                        else:
                            print("connection to [%s] timed out"%node_name)
                        return False
                    except ValueError:
                        print("unknown network error contacting node: %s"%(str(e)))
                if max_count and count >= max_count:
                    break
                time.sleep(1.0)
        except KeyboardInterrupt:
            pass
                
        if details and count > 1:
            print("ping average: %fms"%(acc/count))
        return True

    def feedback(self):
        while not rospy.is_shutdown():
            ping = check_.rosnode_ping(self.node_name, max_count=3,details=False)#max_count为允许ping通次数
            if not ping and self.find != None:
                self.find = None
                for i in range(3):
                    self.stopMoving() #防止速度置零信息丢失发布3次
                rospy.logwarn('Node Connection Failed!')
            time.sleep(1.0)

if __name__ == '__main__':
    rospy.init_node("check_node")
    check_ = Node_Detecion()
    check_.feedback()
    rospy.spin()
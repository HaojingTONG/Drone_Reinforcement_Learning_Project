## 1. lack save fuction
## 2. optitrack 
import multiprocessing
import time
import threading
from NatNetClient import NatNetClient   #optitrack
import numpy as np

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.motion_commander import MotionCommander

DEFAULT_HEIGHT = 0.5
URI0 = 'radio://0/80/2M/E7E7E7E7E7'
URI1 = 'radio://0/83/2M/E7E7E7E7E6'
URI2 = 'radio://0/82/2M/E7E7E7E7E7'
uris = {
    URI0,
    # URI1,
    # URI2,
}

optiTrackID = [1, 2, 3] # drone's ID in optiTrack
pos_ot = np.zeros((3, 3)) # 3D position from optiTrack
att_ot = np.zeros((3, 4)) # 3D attitude from optiTrack
FLYING = True
BEGIN = time.time()



# This is a callback function that gets connected to the NatNet client and called once per mocap frame.
def receiveNewFrame( frameNumber, markerSetCount, unlabeledMarkersCount, rigidBodyCount, skeletonCount,
                    labeledMarkerCount, latency, timecode, timecodeSub, timestamp, isRecording, trackedModelsChanged ):
	pass

# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receiveRigidBodyFrame( id, position, rotation ):
    global pos_ot, att_ot
    if optiTrackID.count(id)==1:
        pos_ot[optiTrackID.index(id),:] = position
        att_ot[optiTrackID.index(id),:] = rotation

streamingClient = NatNetClient() # Create a new NatNet client
streamingClient.newFrameListener = receiveNewFrame
streamingClient.rigidBodyListener = receiveRigidBodyFrame
streamingClient.run() # Run perpetually on a separate thread.

class CrazyFliem:
    def __init__(self, uri):
        self.uri = uri
        if self.uri =='radio://0/80/2M/E7E7E7E7E5':
            self.ID = 1
            self.file = open('./dat01.csv', 'w')
        elif self.uri =='radio://0/83/2M/E7E7E7E7E6':
            self.ID = 2
            self.file = open('./dat02.csv', 'w')
        else:
            self.ID = 3
            self.file = open('./dat03.csv', 'w')

        self._lg_dist = LogConfig(name='ranging',period_in_ms=10)
        self._lg_dist.add_variable('swarmstate.swaGz','float')
        self._lg_dist.add_variable('swarmstate.swah','float')
        self._lg_dist.add_variable('swarmstate.swaVx','float')
        self._lg_dist.add_variable('swarmstate.swaVy','float')
        self._lg_dist.add_variable('ranging.distance0','uint16_t')
        self._lg_dist.add_variable('ranging.distance1','uint16_t')
        self._lg_dist.add_variable('ranging.distance2','uint16_t')
        self._lg_dist.add_variable('ranging.distance3','uint16_t')
        self._lg_dist.add_variable('ranging.distance4','uint16_t')
        print("scan",self.uri)

    def simple_param_async(self, scf, num):
        global FLYING
        cf = scf.cf
        groupstr = 'relative_ctrl'
        namestr = 'keepFlying'
        full_name = groupstr + "." +namestr
        cf.param.add_update_callback(group=groupstr, name=namestr,
                                                                         cb=self.param_stab_est_callback)
        cf.param.set_value(full_name, num)
        if num == 0:
            FLYING = False
    
    def param_stab_est_callback(self, name, value):
        print('The crazyflie has parameter ' + name + ' set at number: ' + value)

    def log_stab_callback(self, timestamp, data, logconf):
        global pos_ot, att_ot
        print('[%s] [%d][%s]: %s' % (self.uri, timestamp, logconf.name, data))
        self.file.write('{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}\n'.format(timestamp,\
            data['swarmstate.swaGz'], data['swarmstate.swah'],data['swarmstate.swaVx'], data['swarmstate.swaVy'], data['ranging.distance0'], data['ranging.distance1'], data['ranging.distance2'], data['ranging.distance3'], data['ranging.distance4'], \
            pos_ot[self.ID-1][0], pos_ot[self.ID-1][1], pos_ot[self.ID-1][2], att_ot[self.ID-1][0], att_ot[self.ID-1][1], att_ot[self.ID-1][2], att_ot[self.ID-1][3]))   


    def process(self):
            with SyncCrazyflie(self.uri, cf=Crazyflie(rw_cache='./cache')) as scf:
                print("connected",self.uri)
                self.scf = scf
                self.cf = self.scf.cf
                self.cf.log.add_config(self._lg_dist)
                self._lg_dist.data_received_cb.add_callback(self.log_stab_callback)
                self._lg_dist.start()
                ####  control #####
                with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
                    # time.sleep(1)
                    # mc.forward(0.5)
                    # time.sleep(2)
                    # mc.turn_left(180)
                    # time.sleep(1)
                    # mc.forward(0.5)
                    time.sleep(1)
                    mc.circle_left(0.5)
                    # mc.start_turn_right
                    time.sleep(2)
                self._lg_dist.stop()

                

if __name__ == '__main__':
    # Initialize the low-level drives
    cflib.crtp.init_drivers()
    flies = []
    thread_list = []
    data_list = []
    for i in uris:
        flie = CrazyFliem(i)
        flies.append(flie)

    for flie in flies:
        t = threading.Thread(target=flie.process)
        t.start()
        thread_list.append(t)

    for t in thread_list:
        t.join()
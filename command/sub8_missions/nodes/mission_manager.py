#!/usr/bin/env python 
import os
import yaml
import copy
import signal
import pprint
import traceback
import exceptions
from operator import itemgetter
from twisted.internet import defer, reactor

import rospy
import txros
from sub8 import tx_sub
import missions.mission_library as mission_library

# TODO: Should be handled as arguments which are provided via launch
manifest_path = os.path.abspath(os.path.join(__file__, "../..")) + '/missions/manifest.yaml'
mission_path = os.path.abspath(os.path.join(__file__, "../..")) + '/missions/mission_library/'

class Sub8Missionlet():
    def __init__(self, d, functor=None):
        # Fields provided by the dictionary are: 
        # name, eta, timeout, rank, points, args 
        self.__dict__ = d
        self.functor = functor

    def __str__(self):
        return  'Mission: ' + self.name + '\n'\
                '   Rank: ' + str(self.rank) + '\n'\
                '   ETA: ' + str(self.eta) + ' secs\n'\
                '   Timeout: ' + str(self.timeout) + ' secs\n'\
                '   Points: ' + str(self.points)


class Sub8MissionManager():
    # TODO: Init stage should handle fetching sub_singleton
    # We should be able to change parameters set in that sub object from a Sub8MissionManager instance
    def __init__(self):
        #self.start_time = rospy.get_rostime()
        #rospy.loginfo("Sub8 Mission Manager started at %i %i", self.start_time.secs, self.start_time.nsecs)

        self.manifest_path = manifest_path
        missionlet_queue = self.parse_manifest()
        self.mission_list = self.assemble_mission(missionlet_queue)

    def __str__(self):
        # Function should return mission manifest, along
        # with estimates timings for each 
       return 'I am a mission manager'

    def parse_manifest(self):
        # Get list of available missions
        missionlet_library = []
        missionlet_queue = []

        stream = open(manifest_path, 'r')
        manifest = yaml.safe_load(stream)
        stream.close()

        desired_missionalets = [Sub8Missionlet(mission['mission']) for mission in manifest]

        for missionlet in os.listdir(mission_path):
            if not missionlet.startswith('_') and not missionlet.endswith('pyc', 3):
                missionlet_library.append(missionlet.split(".")[0])

        for missionlet in desired_missionalets:
            if missionlet.name in missionlet_library:
                try:
                    missionlet.functor = getattr(mission_library, missionlet.name)
                    missionlet_queue.append(missionlet)
                except AttributeError:
                    traceback.print_exc()
            else:
                # TODO: Remove this, it should be handled as ROS log error
                print "'{}' is not an available mission".format(mission)

        return missionlet_queue

    def assemble_mission(self, missionlet_list, key='rank'):
        # Missions will have a timeout period, and will fix roll and pitch after each mission
        # EX: level_off -> mission_1 -> timeout + level_off -> mission_2 ...
        try:
            level_off = Sub8Missionlet({
                'name'   : 'level_off',
                'rank'   : '0',
                'eta'    : '30',
                'timeout': '30',
                'points' : '0'
            },
            getattr(mission_library, 'level_off')
            )
        except AttributeError:
            print 'assemble_mission: Level mission was not found'
            return missionlet_list

        #missionlet_queue = self._sort(missionlet_list, key)

        assembled_mission = []
        assembled_mission.append(level_off)

        for missionlet in missionlet_list:
            assembled_mission.extend([missionlet, copy.deepcopy(level_off)])

        # Debug
        #for i in assembled_mission:
        #    print i.name + ' functor addr: ', i.functor.run

        return assembled_mission

    def _handle_ex(self, exception):
        # TODO: Handle executions 
        print '------ Exception Traceback ------'
        print exception.printTraceback()
        print '---------------------------------'

    def _pause(self):
        # Future functionality; will pause defered
        pass

    def _sort(self, missionlet_list, key):
        # Arrange missions by key. i.e. rank, eta, points
        try:
            if key is 'custom':
                return mission_list
            else:
                return sorted(missionlet_list, key=itemgetter(str(key)))
        except AttributeError:
            print '_sort : Not valid attribute'
            return missionlet_list

    def _start(self):
        signal.signal(signal.SIGINT, lambda signum, frame: reactor.callFromThread(task.cancel))
        task = main(self).addErrback(lambda fail: fail.trap(defer.CancelledError))

    @txros.util.cancellableInlineCallbacks
    def run(self):
        nh_args = yield txros.NodeHandle.from_argv_with_remaining('sub8_mission')
        nh, args = nh_args
        sub = yield tx_sub.get_sub(nh)
        yield txros.util.wall_sleep(1.0)
        yield sub.last_pose()

        for missionlet in self.mission_list:
            yield txros.util.wrap_timeout(missionlet.functor.run(sub).addErrback(self._handle_ex), int(missionlet.timeout))
        txros.util.wall_sleep(1.0)


@txros.util.cancellableInlineCallbacks
def main(sub_man):
    try:
       yield sub_man.run()
    finally:
        #end_time = rospy.get_rostime()
        print 'Finished Execution'
        reactor.stop()

if __name__=='__main__':
    sub_man = Sub8MissionManager()
    reactor.callWhenRunning(sub_man._start)
    reactor.run()

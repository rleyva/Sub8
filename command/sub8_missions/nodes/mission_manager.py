#!/usr/bin/env python 
import os
import signal
import traceback
from twisted.internet import defer, reactor
import exceptions
import txros
from sub8 import tx_sub
import missions.mission_library as mission_library
import yaml

# Parse 
class Sub8Missionlet():
    def __init__(self, d):
        # Fields provided by the dictionary are: 
        # name, eta, timeout, rank, points, args 
        self.__dict__ = d
        self.functor = None

    def __str__(self):
        return  'Mission: ' + self.name + '\n'\
                '   Rank: ' + str(self.rank) + '\n'\
                '   ETA: ' + str(self.eta) + ' secs\n'\
                '   Timeout: ' + str(self.timeout) + ' secs\n'\
                '   Points: ' + str(self.points)

class Sub8MissionManager():
    def __init__(self, manifest_path, mission_list):
        # TODO: parse all missions in the missions directory & manifest yaml
        # Append missionlets to mission list
        # Run mission checks
        self.manifest_path = manifest_path
        self.mission_list = mission_list

    def __str__(self):
        # Function should return mission manifest, along
        # with estimates timings for each 
        pass

    def assemble_mission(self):
        # TODO: Add roll & pitch corrections along with other things to 
        # make missions smoother.
        pass

    def handle_ex(self, exception):
        # TODO: Handle executions 
        pass

    def pause(self):
        # Future functionality 
        pass

    def run(self):
        # 
        pass

def parse_mission(manifest_pth):
    stream = open(manifest_pth, 'r')
    manifest = yaml.safe_load(stream)
    stream.close()
    return [Sub8Missionlet(mission['mission']) for mission in manifest]

@txros.util.cancellableInlineCallbacks
def main():
    try:
        # TODO: Handle this in ROS launch  
        nh_args = yield txros.NodeHandle.from_argv_with_remaining('sub8_mission')

        todo_list = []
        mission_avail = []
        manifest_pth = os.path.abspath(os.path.join(__file__, "../..")) + '/missions/manifest.yaml'
        mission_pth = os.path.abspath(os.path.join(__file__, "../..")) + '/missions/mission_library/'
        mlist_ = parse_mission(manifest_pth)

        for mission in os.listdir(mission_pth):
            if not mission.startswith('_') and not mission.endswith('pyc',3):
                mission_avail.append(mission.split(".")[0])

        print 'Available Missions: ' + '\n'
        print mission_avail 

        for mission in mlist_:
            if mission.name in mission_avail:
                try:
                    mission.functor = getattr(mission_library, mission.name)
                    todo_list.append(mission)
                except AttributeError:
                     traceback.print_exc()
            else:
                print "'{}' is not an available mission".format(mission)

        # Now we're ready for fun!
        nh, args = nh_args
        sub = yield tx_sub.get_sub(nh)
        yield txros.util.wall_sleep(1.0)

        yield sub.last_pose()

        for i in range(3):
            for chore in todo_list:
                yield txros.util.wrap_timeout(chore.functor.run(sub), int(chore.timeout))

        yield txros.util.wall_sleep(1.0)

    #except Exception:
    #    print traceback.print_exec()

    finally:
        print 'Finished Execution'
        reactor.stop()

def _start():
    signal.signal(signal.SIGINT, lambda signum, frame: reactor.callFromThread(task.cancel))
    task = main().addErrback(lambda fail: fail.trap(defer.CancelledError))

if __name__=='__main__':
    reactor.callWhenRunning(_start)
    reactor.run()

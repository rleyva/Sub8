#!/usr/bin/env python 
import os
import signal
import traceback
from twisted.internet import defer, reactor
import exceptions
import txros
from sub8 import tx_sub
import missions.mission_library as mission_library
import xml.etree.ElementTree as ET

class Sub8Missionlet():
    # Only for organization
    def __init__(self, name, rank, eta, timeout, points):
        self.name = name
        self.rank = rank
        self.eta = eta
        self.timeout = timeout
        self.points = points
        self.functor = None

    def __str__(self):
        return 'Mission: ' + self.name + '\n'\
                '   Rank: ' + self.rank + '\n'\
                '   ETA: ' + self.eta + '\n'\
                '   Timeout: ' + self.timeout + '\n'\
                '   Points: ' + self.points

def parse_mission(path):
    parsed_m = []
    tree = ET.parse(path)
    tr_ = tree.getroot()

    for i, s_mission in enumerate(tr_.iter('mission')):
        parsed_m.append(Sub8Missionlet(s_mission.attrib['name'], tr_[i][0].text,\
                        tr_[i][1].text, tr_[i][2].text, tr_[i][3].text)
                       )
    return parsed_m

@txros.util.cancellableInlineCallbacks
def main():
    try:
        # TODO: Handle this in ROS launch  
        nh_args = yield txros.NodeHandle.from_argv_with_remaining('sub8_mission')

        todo_list = []
        mission_avail = []
        manifest_pth = os.path.abspath(os.path.join(__file__, "../..")) + '/missions/manifest.xml'
        mission_pth = os.path.abspath(os.path.join(__file__, "../..")) + '/missions/mission_library/'
        mlist_ = parse_mission(manifest_pth)

        for mission in os.listdir(mission_pth):
            if not mission.startswith('_') and not mission.endswith('pyc',3):
                mission_avail.append(mission.split(".")[0])

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

    except Exception:
        print traceback.print_exec()

    finally:
        print 'Finished Execution'
        reactor.stop()

def _start():
    signal.signal(signal.SIGINT, lambda signum, frame: reactor.callFromThread(task.cancel))
    task = main().addErrback(lambda fail: fail.trap(defer.CancelledError))

if __name__=='__main__':
    reactor.callWhenRunning(_start)
    reactor.run()

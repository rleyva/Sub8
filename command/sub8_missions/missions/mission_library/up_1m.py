from txros import util 

@util.cancellableInlineCallbacks
def run(sub_singleton):
    print 'Rising 1m'
    yield sub_singleton.move.up(1).go()
    print 'Done!'

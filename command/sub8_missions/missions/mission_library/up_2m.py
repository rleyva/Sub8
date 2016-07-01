from txros import util 

@util.cancellableInlineCallbacks
def run(sub_singleton):
    print 'Rising 2m'
    yield sub_singleton.move.up(2).go()
    print 'Done!'

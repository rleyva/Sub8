from txros import util


@util.cancellableInlineCallbacks
def run(sub_singleton):
    print "Dropping 1m"
    yield sub_singleton.move.down(1.0).go()
    print "Done!"

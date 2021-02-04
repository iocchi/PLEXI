#
# wait action implementation
#

import sys, time

import rospy

sys.path.append("../../../rospy")
from actionproxy import ActionProxy

ACTION_NAME = 'wait'

class WaitActionProxy(ActionProxy):

    def __init__(self, actionname, rospytime=True):
        ActionProxy.__init__(self, actionname)
        self.rospytime = rospytime

    def interrupt(self):
        self.interrupted = True   # overwrite default behavior: end

    def resume(self):
        self.interrupted = False  # overwrite default behavior: start

    def action_thread(self, params):
        if not self.interrupted:
            cnt = 0
        self.interrupted = False
        try:
            v = params.split('_')
            timeout = int(v[0])
        except:
            print("Action %s - Error in parsing params %s" %(self.actionname,params))
            timeout = 10
        while self.do_run and cnt<timeout:
            if not self.interrupted:
                cnt += 1
                print("Action %s - counter: %d/%d" %(self.actionname,cnt,timeout))
            if self.rospytime:
                rospy.sleep(1)
            else:
                time.sleep(1)


if __name__ == "__main__":

    params = None
    if (len(sys.argv)>1):
        params = sys.argv[1]

    wait = WaitActionProxy(ACTION_NAME)
    
    if params is not None:
        wait.execute(params)  # blocking, CTRL-C to interrupt
    else:
        wait.run_server()     # blocking, CTRL-C to interrupt


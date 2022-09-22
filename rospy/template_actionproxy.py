# template action

import sys

import rospy

#sys.path.append("....")                # <--- folder containing ActionProxy
from actionproxy import ActionProxy

ACTION_NAME = 'template'                # <--- action name
ACTION_NAME_ALIASES = []                # <--- other action names (aliases)

class TemplateActionProxy(ActionProxy): # <--- action class

    def __init__(self, actionname, actionnamealiases):
        ActionProxy.__init__(self, actionname, actionnamealiases)
                                        # <--- action init

    def __del__(self):
        ActionProxy.__del__(self)
                                        # <--- action del


    #def interrupt(self):
    #    ActionProxy.end(self)          # <--- action interrupt = end

    #def resume(self):
    #    ActionProxy.resume(self)       # <--- action resume = start with same params


    def action_thread(self, params):
        v = params.split('_')           # <--- action params

        while self.do_run:

            if not self.interrupted:
                ...                     # <--- action step

            rospy.sleep(0.25)

                                        # <--- action end


if __name__ == "__main__":

    params = None
    if (len(sys.argv)>1):
        params = sys.argv[1]

    a = TemplateActionProxy(ACTION_NAME, ACTION_NAME_ALIASES)    # <--- action class
    
    if params is not None:
        a.execute(params)  # blocking, CTRL-C to interrupt
    else:
        a.run_server()     # blocking, CTRL-C to interrupt


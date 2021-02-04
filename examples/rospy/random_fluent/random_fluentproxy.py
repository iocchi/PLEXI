# random fluent implementation

import sys, random

import rospy

sys.path.append("../../../rospy")
from fluentproxy import FluentProxy

FLUENT_NAME = 'random'

class RandomFluentProxy(FluentProxy):

    def __init__(self, fluentnane):
        FluentProxy.__init__(self, fluentnane)

    def __del__(self):
        FluentProxy.__del__(self)


    def fluent_thread(self, params):

        while self.do_run:
            value = random.choice([1,0,-1])
            self.setValue(value)
            rospy.sleep(1.0)





if __name__ == "__main__":

    params = ''
    if (len(sys.argv)>1):
        params = sys.argv[1]

    t = RandomFluentProxy(FLUENT_NAME)
    t.execute(params)  # blocking, CTRL-C to interrupt



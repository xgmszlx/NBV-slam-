#!/usr/bin/env python3
"""Start/stop service gate for active exploration experiments."""

import rospy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, TriggerResponse


class ExplorationManager:
    def __init__(self):
        self.active = False
        self.state_pub = rospy.Publisher("/hnbv/exploration_active", Bool, queue_size=1, latch=True)
        self.start_srv = rospy.Service("/hnbv/start", Trigger, self.start)
        self.stop_srv = rospy.Service("/hnbv/stop", Trigger, self.stop)
        self.publish_state()

    def publish_state(self):
        self.state_pub.publish(Bool(data=self.active))

    def start(self, _request):
        self.active = True
        self.publish_state()
        return TriggerResponse(success=True, message="HNBV exploration started")

    def stop(self, _request):
        self.active = False
        self.publish_state()
        return TriggerResponse(success=True, message="HNBV exploration stopped")


def main():
    rospy.init_node("exploration_manager")
    ExplorationManager()
    rospy.loginfo("exploration_manager services ready: /hnbv/start and /hnbv/stop")
    rospy.spin()


if __name__ == "__main__":
    main()


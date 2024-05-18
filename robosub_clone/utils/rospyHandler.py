import rospy
from .topicService import TopicService


class RosHandler:
    def __init__(self):
        self.rate = 1
        self.connected = False

    def connect(self, node: str, rate: int):
        rospy.init_node(node, anonymous=True)
        self.rate = rospy.Rate(rate)
        self.connected = True
        rospy.loginfo("Rospy is up ...")
        rospy.spin()

    def disconnect(self):
        if self.connected:
            rospy.loginfo("shutting down rospy ...")
            rospy.signal_shutdown("disconnect")
            self.connected = False

    @staticmethod
    def topic_publisher(topic: TopicService):
        pub = rospy.Publisher(topic.get_name(), topic.get_type(), queue_size=10)
        pub.publish(topic.get_data())
        # print("published to " + topic.get_name())

    @staticmethod
    def topic_subscriber(topic: TopicService, function=None):
        if function == None:
            function = topic.set_data
        rospy.Subscriber(topic.get_name(), topic.get_type(), function)

    @staticmethod
    def service_caller(service: TopicService, timeout=30):
        try:
            srv = service.get_name()
            typ = service.get_type()
            data = service.get_data()

            rospy.loginfo(f"waiting for ROS service:{srv}")
            rospy.wait_for_service(srv, timeout=timeout)
            rospy.loginfo(f"ROS service is up:{srv}")
            call_srv = rospy.ServiceProxy(srv, typ)
            return call_srv(data)
        except rospy.ROSException as e:
            print("ROS ERROR:", e)
        except rospy.ROSInternalException as e:
            print("ROS ERROR:", e)
        except KeyError as e:
            print("ERROR:", e)
        return None

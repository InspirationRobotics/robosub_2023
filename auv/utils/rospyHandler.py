"""
To handle ROS topics; this file was made so that pix_standalone.py would not be as long
"""

import rospy
from .topicService import TopicService # Handles simple things like getting ROS topic name, type (prebuilt functions)


class RosHandler:
    """
    Class to handle ROS topics
    """
    def __init__(self):
        """
        Initialize a ROS handler object with default values
        """
        self.rate = 1
        self.connected = False

    def connect(self, node: str, rate: int):
        """
        To connect to a ROS node

        Args:
            node (str): Name of the node to connect to
            rate (int): Publishing rate in Hz
        """
        rospy.init_node(node, anonymous=True)
        self.rate = rospy.Rate(rate)
        self.connected = True
        rospy.loginfo("Rospy is up ...")
        rospy.spin() # spin() is to keep node running until shut down from an external place

    def disconnect(self):
        """Disconnect from the ROS node"""
        if self.connected:
            rospy.loginfo("shutting down rospy ...")
            rospy.signal_shutdown("disconnect")
            self.connected = False

    @staticmethod # Static method for functions so that the class instance does not have to be present for the function to be run
    def topic_publisher(topic: TopicService):
        """
        Note: The "topic: TopicService" simply denotes that topic should be an 
        instance of te TopicService class, it does not serve an actual functionality

        To publish data to a ROS publisher

        Args:
            topic (TopicService): the topic to publish to
        """
        pub = rospy.Publisher(topic.get_name(), topic.get_type(), queue_size=10)
        pub.publish(topic.get_data())
        # print("published to " + topic.get_name())

    @staticmethod
    def topic_subscriber(topic: TopicService, function=None):
        """
        To subscribe to a specified ROS topic

        Args:
            topic (TopicService): The topic to subscribe to
            function (optional): A function to handle the received data
        """
        if function == None:
            function = topic.set_data
        rospy.Subscriber(topic.get_name(), topic.get_type(), function)

    @staticmethod
    def service_caller(service: TopicService, timeout=30):
        """
        Call a ROS service

        Args:
            service (TopicService): the service to call (ROS topic to call the service from)
            timeout (int, optional): the time to wait for the service
        """
        try:
            # Get the name, type, and data from the ROS topic 
            srv = service.get_name()
            typ = service.get_type()
            data = service.get_data()

            # Wait for the ROS service
            rospy.loginfo(f"waiting for ROS service:{srv}")
            rospy.wait_for_service(srv, timeout=timeout)
            rospy.loginfo(f"ROS service is up:{srv}")

            # Create a proxy object for the ROS service
            call_srv = rospy.ServiceProxy(srv, typ)
            # Put the data into the service, and return the result
            return call_srv(data)
        
        # Handle exceptions
        except rospy.ROSException as e:
            print("ROS ERROR:", e)
        except rospy.ROSInternalException as e:
            print("ROS ERROR:", e)
        except KeyError as e:
            print("ERROR:", e)
        return None

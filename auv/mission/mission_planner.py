import glob
import importlib
import inspect
import json
import os
import threading
import time

import networkx as nx

from ..utils.deviceHelper import variables


def import_mission_by_name(mission_name):
    """Imports a mission by name and returns the mission class

    Args:
        mission_name (str): Name of the mission to import
    """
    try:
        # Import the mission
        module = importlib.import_module(f"auv.mission.{mission_name}")

        # get the class element of the mission
        return inspect.getmembers(module, inspect.isclass)[0][1]
    except Exception as e:
        print(f"Failed to import mission {mission_name}")
        print(e)
        return None


def get_mission_list():
    """Returns a list of all missions in the mission folder"""
    file_path = os.path.dirname(os.path.realpath(__file__))

    # Get all files in this folder ending with *_mission.py
    files = glob.glob(os.path.join(file_path, "*_mission.py"))

    # Get the name of the mission from the file name
    return [os.path.basename(f).split(".py")[0] for f in files]


class Node:
    def __init__(self, mission_name, config):
        self.name = mission_name
        self.mission_class = import_mission_by_name(mission_name)
        self.mission = None  # mission object

        self.next_nodes = config.get("next_nodes", None)  # nodes to start after this one
        self.timeout = config.get("timeout", None)  # end the node if it hasn't succeeded after a given time

        self.return_code = -1  # -1 = not started, 0 = success, 1 = failed
        self.start_time = 0
        self.active = False
        self.to_thread = None

    def get_next(self):
        return self.next

    def init(self):
        try:
            self.mission = self.mission(**variables)
            self.active = True
            return True
        except Exception as e:
            print(f"Failed to initialize mission {self.mission_class}, disabling Node", f"{e}", sep="\n")
            self.active = False
            self.return_code = 1
            return False

    def timeout_check(self):
        if self.timeout is None or self.timeout < 0:
            return

        elif self.start_time + self.timeout < time.time():
            print(f"Mission {self.mission_class} timed out")
            self.end(1)

    def start(self):
        self.start_time = time.time()
        self.active = True

        # check if the mission is initialized
        if self.mission is None:
            if not self.init():
                return

        # start the mission
        self.mission.run()
        self.to_thread = threading.Timer(1.0, self.timeout_check)

    def end(self, code):
        self.return_code = code
        self.active = False

        # TODO properly end the mission
        return


class MissionPlanner:
    def __init__(self, mission_path):
        self.nodes = self.load_mission_plan(mission_path)
        self.G = nx.DiGraph()

    def load_mission_plan(self, mission_path):
        # load the json config file
        with open(mission_path, "r") as f:
            config = json.load(f)

        # for each key, create a node
        nodes = {}

        for key in config.keys():
            # get the mission class
            mission_class = import_mission_by_name(key)

            # create the node
            nodes[key] = Node(mission_class, config[key])
            self.G.add_node(key)

        for node in nodes.values():
            # check if the node has a next node
            if node.next_nodes is None:
                continue

            # check if the next node is a list
            if not isinstance(node.next_nodes, list):
                node.next_nodes = [node.next_nodes]

            # check if the next node is a valid node
            for next_node_name in node.next_nodes:
                if next_node_name not in nodes.keys():
                    print(f"Node {node.name}: next Node {next_node_name} is not a valid node, removing it")
                    node.next_nodes.remove(next_node_name)
                else:
                    self.G.add_edge(node.name, next_node_name)

        if not nx.is_directed_acyclic_graph(self.G):
            print(f"[WARNING] Mission graph contains a loop, this may cause problems")

        return nodes

    def viz_graph(self):
        from matplotlib import pyplot as plt

        # check for loops

        nx.draw(self.G, with_labels=True)
        plt.show()


if __name__ == "__main__":
    missions_names = get_mission_list()
    mission_classes = [import_mission_by_name(m) for m in missions_names]

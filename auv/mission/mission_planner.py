import glob
import importlib
import inspect
import yaml
import os
import threading
import time

import networkx as nx

from ..utils.deviceHelper import variables
from ..device.modems import modems_api


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
    def __init__(self, name, config):
        self.next_nodes = config.get("next_nodes", None)  # nodes to start after this one
        self.timeout = config.get("timeout", None)  # end the node if it hasn't succeeded after a given time
        self.mission_name = config.get("mission_name", None)  # name of the mission to run

        if self.mission_name is None:
            print(f"Node {name} has no mission name, disabling node")
            self.active = False
            self.return_code = 1
            return

        self.name = name
        self.mission_class = import_mission_by_name(self.mission_name)
        self.mission = None  # mission object

        self.return_code = -1  # -1 = not started, 0 = success, 1 = failed, 2 = timeout
        self.start_time = 0
        self.active = False
        self.if_condition = None

        self.to_thread = None
        self.run_thread = None

    def init(self):
        try:
            if self.mission_class is None:
                raise Exception(f"Mission {self.mission_name} does not exist in list: {get_mission_list()}, disabling Node")

            self.mission = self.mission_class(**variables)
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
            self.end(2)

    def start(self):
        self.start_time = time.time()
        self.active = True

        # check if the mission is initialized
        if self.mission is None:
            if not self.init():
                return False

        # start the mission
        self.run_thread = threading.Thread(target=self.mission.run)
        self.to_thread = threading.Timer(1.0, self.timeout_check)

        self.to_thread.start()
        self.run_thread.start()
        return True

    def end(self, code):
        self.return_code = code
        self.active = False

        # stop the mission
        self.mission.stop()
        return


class MissionPlanner:
    def __init__(self, mission_path):
        self.G = nx.DiGraph()
        self.nodes = self.load_mission_plan(mission_path)

    def check_if_condition(self, node: Node):
        if_condition = getattr(node, "if_condition", None)
        if if_condition is None:
            return True

        # eval the if condition
        # CAREFUL: this is a security risk
        return eval(if_condition)

    def load_mission_plan(self, mission_path):
        # load the yaml config file
        with open(mission_path, "r") as f:
            config = yaml.safe_load(f)

        nodes = {}
        # create the nodes
        for name in config.keys():
            # create the node
            node = Node(name, config[name])
            setattr(self, name, node)
            nodes[name] = node
            self.G.add_node(name)

        for node in nodes.values():
            # check if the node has a next node
            if node.next_nodes is None:
                continue

            # check if the next node is a list
            if not isinstance(node.next_nodes, list):
                node.next_nodes = [node.next_nodes]

            # check if the next node is a valid node
            for next_node in node.next_nodes:
                # check if the next node is an if statement
                # this statement will be used to determine if the node should be run
                if isinstance(next_node, dict):
                    next_node_name = list(next_node.keys())[0]
                    if_condition = list(next_node.values())[0]
                    nodes[next_node_name].if_condition = if_condition
                elif isinstance(next_node, str):
                    next_node_name = next_node
                else:
                    next_node_name = None

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

        nx.draw(self.G, with_labels=True, font_weight="bold")
        plt.show()

    def get_entry_nodes(self):
        return [node for node_name, node in self.nodes.items() if self.G.in_degree(node_name) == 0]
    
    def run(self):
        entry_nodes = self.get_entry_nodes()
        print(f"Entry nodes: {[node.name for node in entry_nodes]}")
        
        # start the entry nodes
        for node in entry_nodes:
            if self.check_if_condition(node):
                if node.start():
                    node.to_thread.start()


if __name__ == "__main__":
    missionPlanner = MissionPlanner("missions/mission_plan.yaml")
    missionPlanner.run()
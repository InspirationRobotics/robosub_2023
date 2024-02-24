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
        print(f"Failed to import mission {mission_name} with error: {e}")
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
        self.next_nodes_names = config.get(
            "next_nodes", []
        )  # nodes to start after this one
        self.timeout = config.get(
            "timeout", None
        )  # end the node if it hasn't succeeded after a given time
        self.mission_name = config.get(
            "mission_name", None
        )  # name of the mission to run

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

        self.to_thread = None
        self.run_thread = None

    def init(self):
        try:
            if self.mission_class is None:
                raise Exception(
                    "Failed to import mission, check the mission name or syntax error in the mission file"
                )

            self.mission = self.mission_class(**variables)
            return True

        except Exception as e:
            print(f"Failed to initialize mission {self.name}, {e}")
            self.return_code = -1
            self.active = False
            return False

    def timeout_check(self):
        if self.timeout is None or self.timeout < 0:
            return

        if self.start_time + self.timeout < time.time():
            print(f"Mission {self.mission_class} timed out")
            self.end(2)

    def start(self):
        self.start_time = time.time()
        self.active = True

        # check if the mission is initialized
        if self.mission is None:
            if not self.init():
                self.active = False
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

    def check_if_condition(self, next_node_name: str, if_condition=None):
        if if_condition is None:
            return True

        # evaluate the if condition
        # CAREFUL: this is a security risk
        try:
            ret = eval(if_condition)
            if isinstance(ret, bool) or ret in (0, 1):
                return ret

            print(
                f"Failed to eval if condition {if_condition} for next node {next_node_name} got: {ret} instead of a boolean or 0/1"
            )
            return False

        except Exception as e:
            print(
                f"Failed to eval if condition {if_condition} for next node {next_node_name}"
            )
            print(e)
            return False

    def load_mission_plan(self, mission_path):
        # load the yaml config file
        with open(mission_path, "r") as f:
            config = yaml.safe_load(f)

        nodes = {}
        # create the nodes
        for name in config.keys():
            # create the node
            node = Node(name, config[name])
            nodes[name] = node
            self.G.add_node(name)

            # add the node to the mission planner as an attribute
            setattr(self, name, node)

        for node in nodes.values():
            # check if the node has a next node
            if node.next_nodes_names is None or len(node.next_nodes_names) == 0:
                continue

            # check if the next node is a list
            if not isinstance(node.next_nodes_names, list):
                node.next_nodes_names = [node.next_nodes_names]

            # check if the next node is a valid node
            for next_node in node.next_nodes_names:
                # check if the next node is an if statement
                # this statement will be used to determine if the node should be run
                next_node_name = None
                if_condition = None
                if isinstance(next_node, dict):
                    next_node_name = list(next_node.keys())[0]
                    if_condition = list(next_node.values())[0]
                elif isinstance(next_node, str):
                    next_node_name = next_node

                if next_node_name not in nodes.keys():
                    print(
                        f"Node {node.name}: next Node {next_node_name} is not a valid node, removing it"
                    )
                    node.next_nodes_names.remove(next_node_name)
                else:
                    # add the edge to the graph and and the if condition as a label for the edge
                    self.G.add_edge(
                        node.name, next_node_name, if_condition=if_condition
                    )

        if not nx.is_directed_acyclic_graph(self.G):
            print(f"[WARNING] Mission graph contains a cycle, this may cause problems")

        return nodes

    def viz_graph(self):
        from matplotlib import pyplot as plt

        pos = nx.spring_layout(self.G)
        nx.draw(self.G, pos, with_labels=True)

        edges_labels = nx.get_edge_attributes(self.G, "if_condition")
        edges_labels = {k: v for k, v in edges_labels.items() if v is not None}
        nx.draw_networkx_edge_labels(self.G, pos, edge_labels=edges_labels, font_size=8)
        plt.show()

    def get_entry_nodes(self):
        return [
            node
            for node_name, node in self.nodes.items()
            if self.G.in_degree(node_name) == 0
        ]

    def run(self):
        """
        Runs the mission plan
        Technically we could run multiple nodes at the same time,
        but please don't do that as there would be conflicts between the nodes for sure.
        """

        entry_nodes = self.get_entry_nodes()
        print(f"Entry nodes: {[node.name for node in entry_nodes]}")

        # run the mission BFS style
        current_nodes = entry_nodes
        while len(current_nodes) > 0:
            print(f"Running nodes: {[node.name for node in current_nodes]}")

            # start the nodes
            for node in current_nodes:
                node.start()

            # wait for the nodes to finish
            while any([node.active for node in current_nodes]):
                time.sleep(0.1)

            # get the next nodes
            next_nodes = []
            for node in current_nodes:
                for next_node in node.next_nodes_names:
                    next_node_name = None
                    if isinstance(next_node, dict):
                        # check if the if condition is true
                        next_node_name = list(next_node.keys())[0]
                        condition = list(next_node.values())[0]
                        if not self.check_if_condition(next_node_name, condition):
                            continue
                    else:
                        next_node_name = next_node

                    if next_node_name not in self.nodes:
                        print(
                            f"Node {node.name}: next Node {next_node_name} is not a valid node, skipping it"
                        )
                        continue

                    if next_node_name in [n.name for n in next_nodes]:
                        print(
                            f"Node {node.name}: next Node {next_node_name} is already in the next nodes, skipping it"
                        )
                        continue

                    next_nodes.append(self.nodes[next_node_name])

            # prepare next iteration
            current_nodes = next_nodes


if __name__ == "__main__":
    # usage: python3 -m auv.mission.mission_planner -p path_to_mission_plan.yaml -v (optional for visualization)
    import argparse

    parser = argparse.ArgumentParser(description="Run the mission planner")
    parser.add_argument(
        "-p",
        "--plan",
        type=str,
        required=True,
        help="Path to the mission plan yaml file",
    )
    parser.add_argument(
        "-v",
        "--visualize",
        action="store_true",
        help="Visualize the mission plan graph",
        default=False,
    )
    args = parser.parse_args()

    missionPlanner = MissionPlanner(args.plan)

    if args.visualize:
        missionPlanner.viz_graph()

    missionPlanner.run()

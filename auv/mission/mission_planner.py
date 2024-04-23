"""
Handles the sequencing of missions
"""

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
    """
    Imports a mission by name and returns the mission class.

    Args:
        mission_name (str): Name of the mission file

    Returns:
        class: Mission class inside the mission file
    """
    try:
        # Import the mission
        module = importlib.import_module(f"auv.mission.{mission_name}")

        # Get the class that the mission script defines
        return inspect.getmembers(module, inspect.isclass)[0][1]
    except Exception as e:
        print(f"Failed to import mission {mission_name} with error: {e}")
        return None


def get_mission_list():
    """
    Returns a list containing all of the names of the files inside the mission folder 

    Returns:
        list: Containing the names and only the names of the missions for all of the files inside the missions folder
    """
    file_path = os.path.dirname(os.path.realpath(__file__))

    # Get all files in this folder ending with *_mission.py
    files = glob.glob(os.path.join(file_path, "*_mission.py"))

    # Get the name of the mission from the file name, and return each of them
    return [os.path.basename(f).split(".py")[0] for f in files]


class Node:
    """
    Creates a node (spot of execution in the whole run) for each mission
    """
    def __init__(self, name, config):
        """
        Initialize the Node class.

        Args:
            name: The name of the mission
            config: The file to run based on the mission

        The args are a little confusing, so here is an example:
            Node("Torpedoes", torpedo_mission) -> where Torpedoes is the name and torpedo_mission is the config
        """
        # Get the names of the nodes to start after this current nod
        self.next_nodes_names = config.get(
            "next_nodes", []
        )  

        # End the node if it hasn't succeeded after a given amount of time
        self.timeout = config.get(
            "timeout", None
        )

        # Get the name of the mission
        self.mission_name = config.get(
            "mission_name", None
        ) 

        # End node if there is no mission to run
        if self.mission_name is None:
            print(f"Node {name} has no mission name, disabling node")
            self.active = False
            self.return_code = 1
            return

        self.name = name
        self.mission_class = import_mission_by_name(self.mission_name)
        self.mission = None  # Mission object

        self.return_code = -1  # -1 = not started, 0 = success, 1 = failed, 2 = timeout
        self.start_time = 0
        self.active = False

        self.to_thread = None
        self.run_thread = None

    def init(self):
        """
        Initialize the mission by creating an instance of the correct mission class

        Returns:
            bool: Flag indicating whether the mission class was successfully initialized
        """
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
        """
        Check if the mission has exceeded the amount of time given for its completion.
        If this is the case, exit with a return code of 2 (which indicates timing out). 
        """
        if self.timeout is None or self.timeout < 0:
            return

        if self.start_time + self.timeout < time.time():
            print(f"Mission {self.mission_class} timed out")
            self.end(2)

    def start(self):
        """
        Start the mission by creating a thread for that mission class

        Returns: 
            bool: Flag indicating whether the mission has been initialized/started or not
        """
        self.start_time = time.time()
        self.active = True

        # Check if the mission is initialized
        if self.mission is None:
            if not self.init():
                self.active = False
                return False

        # Start the mission
        self.run_thread = threading.Thread(target=self.mission.run)
        self.to_thread = threading.Timer(1.0, self.timeout_check)

        self.to_thread.start()
        self.run_thread.start()
        return True

    def end(self, code):
        """
        End the mission.

        Args:
            code (int): Flag (-1, 0, 1, 2) indicating why the mission was stopped. 
        """
        self.return_code = code
        self.active = False

        # Stop the mission
        self.mission.stop()
        return


class MissionPlanner:
    """
    Handles running the mission plan. Leverages the Node class to run individual missions (nodes). This class essentially creates a 
    graph, with nodes being vertices/points, and creates the sequence of missions by connecting the nodes using "edges" (which are literally 
    just line segments). This flowchart-style graph is then executed.
    """
    def __init__(self, mission_path):
        """
        Initialize the mission planner

        Args:
            mission_path: The path to the .yaml file that stores the mission sequence
        """
        self.G = nx.DiGraph() # Create the self directed graph
        self.nodes = self.load_mission_plan(mission_path) # Load the individual nodes

        # Note that load_mission_plan will also create the edges between the nodes on self.G, but that will not be in self.nodes, but instead present 
        # on the actual graph, self.G. Self.nodes just holds the indivdual instances of each node.

    def check_if_condition(self, next_node_name: str, if_condition=None):
        """
        
        """
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
        """
        Get the pre-sequenced mission plan from the .yaml file. This also creates the mission nodes based on the .yaml file.

        Args: 
            mission_path (str): Path to the .yaml file.

        Returns:
            dict: Stores the corresponding nodes created by sequence and keyed by name of the mission.

            NOTE: This means that nodes will be stored by position of where they appear in the .yaml file, and will already be loaded as
            an instance of the Node class, the nodes having mission-specific parameters. 

            NOTE: This method also adds edges between nodes on the graph, but that is not actually a return value
        """
        """
        This method uses a self-directed graph to create a mission sequence. Basically each node is a vertex, and there are "edges", or line segments,
        that connect the nodes based on which node comes next. If there is a conditional, the "edge" will not be a simple string, but a dictionary,
        with the key representing the name of the next node, and the stored value representing the condition under which we should move "on" the edge
        to the next node. 
        """
        # Open and read the .yaml mission file
        with open(mission_path, "r") as f:
            config = yaml.safe_load(f)

        nodes = {}

        # Create the individual nodes
        for name in config.keys():
            node = Node(name, config[name]) # Create an instance of the node
            nodes[name] = node # Add the node to the nodes dictionary with the key being the name of the mission
            self.G.add_node(name) # Add the node to the graph (functionality of networkx)

            # Add the node to the mission planner as an attribute
            setattr(self, name, node)

        # Create the interconnectiveness between nodes on the graph (by networkx functionality)
        for node in nodes.values():
            # Check if the node has a next node
            if node.next_nodes_names is None or len(node.next_nodes_names) == 0:
                continue

            # Check if the next node is a list
            if not isinstance(node.next_nodes_names, list):
                node.next_nodes_names = [node.next_nodes_names]

            # Check if the next node is a valid node
            for next_node in node.next_nodes_names:
                # Check if the next node is an if statement -- the condition this defines will be used to determine if the node should run
                next_node_name = None
                if_condition = None
                # If the next node is a dictionary, this implies that this is a conditional, in which case get the condition and the name of the next node to 
                # run if this condition is satisfied.
                if isinstance(next_node, dict):
                    next_node_name = list(next_node.keys())[0]
                    if_condition = list(next_node.values())[0]
                # If the node is a simple string, this next node is the only node possible to move from the current node
                elif isinstance(next_node, str):
                    next_node_name = next_node

                # Delete invalid nodes, else add the edges between the nodes and the conditional as the label for the edge if present
                if next_node_name not in nodes.keys():
                    print(
                        f"Node {node.name}: next Node {next_node_name} is not a valid node, removing it"
                    )
                    node.next_nodes_names.remove(next_node_name)
                else:
                    self.G.add_edge(
                        node.name, next_node_name, if_condition=if_condition
                    )

        if not nx.is_directed_acyclic_graph(self.G):
            print(f"[WARNING] Mission graph contains a cycle, this may cause problems")

        return nodes

    def viz_graph(self):
        """
        Visualize the selfd irected graph
        """
        from matplotlib import pyplot as plt

        pos = nx.spring_layout(self.G)
        nx.draw(self.G, pos, with_labels=True)

        edges_labels = nx.get_edge_attributes(self.G, "if_condition")
        edges_labels = {k: v for k, v in edges_labels.items() if v is not None} #
        nx.draw_networkx_edge_labels(self.G, pos, edge_labels=edges_labels, font_size=8)
        plt.show()

    def get_entry_nodes(self):
        """
        Get the first node on the graph

        Returns:
            The node at degree 0, meaning the node that has no incoming edges
        """
        return [
            node
            for node_name, node in self.nodes.items()
            if self.G.in_degree(node_name) == 0
        ]

    def run(self):
        """
        Run the mission planner.
        """
        """
        Technically several nodes could be run at once. 
        However, DO NOT do this as there would (most likely) be conflicts between nodes.
        """

        # Get the entry node(s)
        entry_nodes = self.get_entry_nodes()
        print(f"Entry nodes: {[node.name for node in entry_nodes]}")

        # Run the mission planner BFS (breath-first-search) style
        # This means that start at the root (entry) node and explore each node at the current depth level before moving further
        # The depth level is the distance of a node from the entry/root node

        # Start with the entry node(s)
        current_nodes = entry_nodes 
        while len(current_nodes) > 0:
            print(f"Running nodes: {[node.name for node in current_nodes]}")

            # Start the nodes 
            for node in current_nodes:
                node.start()

            # Wait for the nodes to finish running
            while any([node.active for node in current_nodes]):
                time.sleep(0.1)

            # Obtain the next nodes
            next_nodes = []
            for node in current_nodes:
                for next_node in node.next_nodes_names:
                    next_node_name = None
                    # If the next node is a dictionary, meaning if the node contains a conditional (or more accurately if the edge requires a conditional 
                    # to traverse through the edge to the next node), check if the conditinal is True
                    if isinstance(next_node, dict):
                        next_node_name = list(next_node.keys())[0]
                        condition = list(next_node.values())[0]
                        if not self.check_if_condition(next_node_name, condition):
                            continue

                    # If there is no conditional, immediately set the next node
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

            # Prepare the next iteration of the current nodes
            current_nodes = next_nodes


if __name__ == "__main__":
    # Usage: python3 -m auv.mission.mission_planner -p path_to_mission_plan.yaml -v (optional for visualization)
    import argparse

    # If running the script directly (for testing purposes), get the keyword arguments (kwargs) for the path to the mission planner .yaml 
    # file, and the bool of whether or not to visualize the mission planner graph
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

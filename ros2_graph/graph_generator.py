#! /usr/bin/env python3
# Copyright 2023 Kiwicampus Inc.
#
# Licensed under GNU GENERAL PUBLIC, version 3 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.gnu.org/licenses/gpl-3.0.en.html
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from functools import partial
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node

from .ros_element import (
    ElementType,
    LinkType,
    NoNodeElement,
    NodeElement,
    list_ros_element_2_list_str,
)
from . import ros_cli_utils as rcu
from functools import reduce

ElementNameTypes = Tuple[str, List[str]]
RelatedNodes = Dict[str, List[str]]
ElementRelatedNodes = Dict[str, RelatedNodes]


class GraphGenerator:
    def __init__(self, style_config: Dict[str, any]):
        rclpy.init()
        self.dummy = Node("Graph_generator")
        self.ros_node_info_file = "node_info.txt"
        self.mainNodes: Dict[int, NodeElement] = dict()
        self.nodes: Dict[int, NodeElement] = dict()
        self.topics: Dict[int, NoNodeElement] = dict()
        self.services: Dict[int, NoNodeElement] = dict()
        self.actions: Dict[int, NoNodeElement] = dict()

        self.link_strs = style_config["links_str"]
        self.brackets = style_config["shapes"]

    def new_no_node(
        self, new_element: NoNodeElement, elemt_dict: Dict[int, NoNodeElement]
    ) -> NoNodeElement:
        """!
        Add new_element to the elemt_dict if it not already exist
        """
        new_hash = hash(new_element)
        if new_hash not in elemt_dict:
            elemt_dict[new_hash] = new_element
        return elemt_dict[new_hash]

    def new_action(self, data: Tuple[str]) -> NoNodeElement:
        """! Initialize a NoNodeElement action object form a string tuple (name, namespace, ros_type) or return the already existing action
        @param action data tuples (name, namespace, ros_type)
        @return NoNodeElement action object
        """
        new_action = NoNodeElement(
            name=data[0],
            namespace=data[1],
            ros_type=data[2],
            type=ElementType.ACTION,
            brackets=self.brackets[ElementType.ACTION],
        )
        return self.new_no_node(new_action, self.actions)

    def actions_tuple(self, actions_data: Tuple[Tuple[str]]) -> Tuple[NoNodeElement]:
        """!
        create a group of actions objects from a tuple of data
        @param actions_data actions info in (name, namespace, ros_type) format
        @return a tuple of NoNodeElement action objects
        """
        return tuple(self.new_action(data) for data in actions_data)

    def new_topic(self, name: str, namespace: str, ros_type: str) -> NoNodeElement:
        """! Initialize a NoNodeElement topic object form a string tuple (name, namespace, ros_type) or return the already existing action
        @param topic data tuples (name, namespace, ros_type)
        @return NoNodeElement topic object
        """
        new_topic = NoNodeElement(
            name,
            namespace,
            ros_type=ros_type,
            type=ElementType.TOPIC,
            brackets=self.brackets[ElementType.TOPIC],
        )
        return self.new_no_node(new_topic, self.topics)

    def new_service(self, name: str, namespace: str, ros_type: str) -> NoNodeElement:
        """! Initialize a NoNodeElement service object form a string tuple (name, namespace, ros_type) or return the already existing action
        @param service data tuples (name, namespace, ros_type)
        @return NoNodeElement service object
        """
        new_service = NoNodeElement(
            name,
            namespace,
            ros_type=ros_type,
            type=ElementType.SERVICE,
            brackets=self.brackets[ElementType.SERVICE],
        )
        return self.new_no_node(new_service, self.services)

    def get_actions(self, node: str) -> Dict[str, Tuple[NoNodeElement]]:
        """! Get Action related to some node
        @param node, the node to inspect its actions
        """
        patterns = {
            "action_servers": ("Action Servers:", "Action Clients:"),
            "action_client": ("Action Clients:",),
        }

        rcu.save_ros_node_info(node_name=node, file=self.ros_node_info_file)
        elements = {
            k: self.actions_tuple(
                rcu.get_node_info_block(pattern, file=self.ros_node_info_file)
            )
            for k, pattern in patterns.items()
        }

        rcu.remove_file(self.ros_node_info_file)

        return elements

    def new_node(self, name: str, namespace: str) -> NodeElement:
        """! Create a new node if is not already created given its data
        and return it
        @param data a tuple (name, namespace)
        @return the NodeElement object
        """
        new_node = NodeElement(
            name,
            namespace,
            type=ElementType.NODE,
            brackets=self.brackets[ElementType.NODE],
        )
        node_hash = hash(new_node)
        if node_hash in self.mainNodes:
            return self.mainNodes[node_hash]
        if node_hash not in self.nodes:
            self.nodes[node_hash] = new_node
        return self.nodes[node_hash]

    def nodes_from_data(self, nodes_data: Tuple[Tuple[str]]) -> Tuple[NodeElement]:
        """!
        create a group of nodes objects from a tuple of data
        @param nodes_data: nodes info in (name, namespace) format
        @return a tuple of NodeElement objects
        """
        return tuple((self.new_node(*node) for node in nodes_data))

    def filter_topics(self, topic: Tuple[str, str]) -> bool:
        """! To filter undesired topics
        @param topic tuple(name, type)
        @return False if the topic is not desired
        """
        exclude = [
            "/parameter_events",
            "/rosout",
            "/tf",
            "/cascade_lifecycle_activations",
            "/cascade_lifecycle_states",
            "",
        ]

        blacklist = ("/transition_event", "/_action")

        if topic[0] in exclude:
            return False

        return is_not_in_blacklist(topic, blacklist)

    def get_topics_related_nodes(
        self, topics: List[ElementNameTypes], subscribers: bool
    ) -> Dict[NoNodeElement, List[NodeElement]]:
        """!
        Get publishers or subcribers nodes for the specified topics

        @param topics (list[tuple(name, types list)]) List of topics with names and type
        @param publishers (bool) get publishers or subscribers nodes

        @return dict {topic_name: {type: str, nodes: [nodes_names]}}
        """

        get_endpoints_function = (
            self.dummy.get_publishers_info_by_topic
            if subscribers
            else self.dummy.get_subscriptions_info_by_topic
        )

        topics_and_nodes = {}
        for topic in filter(self.filter_topics, topics):
            name, namespace = rcu.split_full_name(topic[0])
            ros_type = "<br>".join(topic[1])
            topicObj = self.new_topic(name, namespace, ros_type)
            endpoints = get_endpoints_function(topic[0])

            nodes = [
                self.new_node(endpoint.node_name, endpoint.node_namespace)
                for endpoint in endpoints
            ]
            topics_and_nodes[topicObj] = nodes

        return topics_and_nodes

    def get_services_related_nodes(
        self, services: List[ElementNameTypes], clients: bool
    ) -> Dict[NoNodeElement, List[NodeElement]]:
        """! Get servers or clients nodes for the specified services
        @param services list of tuple(name: str, types: list)
        @param clients, True for get clients names, false for server names
        @return dictonary with services and related nodes
        """

        blacklist = [
            "/change_state",
            "/describe_parameters",
            "/get_available_states",
            "/get_available_transitions",
            "/get_parameter_types",
            "/get_parameters",
            "/get_state",
            "/get_transition_graph",
            "/list_parameters",
            "/set_parameters",
            "/set_parameters_atomically",
            "/_action",
        ]

        get_services_function = (
            self.dummy.get_client_names_and_types_by_node
            if clients
            else self.dummy.get_service_names_and_types_by_node
        )

        filter_services = partial(is_not_in_blacklist, blacklist=blacklist)
        services_and_nodes = {
            service[0]: {"type": "<br>".join(service[1]), "nodes": []}
            for service in filter(filter_services, services)
        }

        services_names = services_and_nodes.keys()
        nodes_list = self.dummy.get_node_names_and_namespaces()

        for node in nodes_list:
            services = tuple(map(lambda x: x[0], get_services_function(*node)))
            filtered_services = tuple(
                filter((lambda service: service in services_names), services)
            )
            for service in filtered_services:
                services_and_nodes[service]["nodes"].append(self.new_node(*node))

        services_and_nodesObj = {}
        for service, sub_dict in services_and_nodes.items():
            name, namespace = rcu.split_full_name(service)
            ros_type = sub_dict["type"]
            serviceObj = self.new_service(name, namespace, ros_type)
            services_and_nodesObj[serviceObj] = sub_dict["nodes"]

        return services_and_nodesObj

    def create_links(
        self,
        main_node: NodeElement,
        relations: Dict[NoNodeElement, List[NodeElement]],
        link_type: LinkType,
    ) -> None:
        """! Add links to the nodes from a relations dictionary
        @param relations, dictonary with the relationships
        @link_type: subscription, publisher, service server ...
        """
        inv_link_type = LinkType.inverse_link(link_type)
        link_str1 = self.link_strs[inv_link_type]
        link_str2 = self.link_strs[link_type]
        for linked_element, nodes in relations.items():
            main_node.add_link(linked_element, link_str1, inv_link_type)
            for node in nodes:
                node.add_link(linked_element, link_str2, link_type)

    def get_node_graph(self, node):
        name, namespace = rcu.split_full_name(node)
        newMain = NodeElement(
            name,
            namespace,
            type=ElementType.MAIN,
            brackets=self.brackets[ElementType.MAIN],
        )
        main_hash = hash(newMain)

        if main_hash in self.mainNodes:
            return
        self.mainNodes[main_hash] = newMain

        elements = self.get_actions(node)

        action_clients_data = {
            action: rcu.get_action_related_nodes(action.full_name(), clients=True)
            for action in elements["action_servers"]
        }
        action_clients = {
            k: self.nodes_from_data(v) for k, v in action_clients_data.items()
        }
        action_servers_data = {
            action: rcu.get_action_related_nodes(action.full_name(), clients=False)
            for action in elements["action_client"]
        }
        action_servers = {
            k: self.nodes_from_data(v) for k, v in action_servers_data.items()
        }
        subscribers = self.dummy.get_subscriber_names_and_types_by_node(name, namespace)
        publishers = self.dummy.get_publisher_names_and_types_by_node(name, namespace)
        services_server = self.dummy.get_service_names_and_types_by_node(
            name, namespace
        )
        services_client = self.dummy.get_client_names_and_types_by_node(name, namespace)

        topics_subscribers = self.get_topics_related_nodes(
            subscribers, subscribers=True
        )
        topics_publishers = self.get_topics_related_nodes(publishers, subscribers=False)
        service_clients = self.get_services_related_nodes(services_server, clients=True)
        service_servers = self.get_services_related_nodes(
            services_client, clients=False
        )

        self.create_links(
            main_node=newMain,
            relations=topics_subscribers,
            link_type=LinkType.TOPIC_SUBSCRIBER,
        )
        self.create_links(
            main_node=newMain,
            relations=topics_publishers,
            link_type=LinkType.TOPIC_PUBLISHER,
        )
        self.create_links(
            main_node=newMain,
            relations=service_clients,
            link_type=LinkType.SERVICE_CLIENT,
        )
        self.create_links(
            main_node=newMain,
            relations=service_servers,
            link_type=LinkType.SERVICE_SERVER,
        )
        self.create_links(
            main_node=newMain,
            relations=action_clients,
            link_type=LinkType.ACTION_CLIENT,
        )
        self.create_links(
            main_node=newMain,
            relations=action_servers,
            link_type=LinkType.ACTION_SERVER,
        )

    def get_nodes_mermaid(
        self, nodes: Dict[int, NodeElement]
    ) -> Tuple[str, Dict[LinkType, List[str]]]:
        nodes_list = [str(val) for val in nodes.values()]
        nodes_mermaid = "\n".join(nodes_list)

        links_mermaid = {
            name: reduce(
                lambda mermaid_lines, node: mermaid_lines + node.get_links_str(name),
                nodes.values(),
                list(),
            )
            for name in LinkType
        }
        return nodes_mermaid, links_mermaid

    def get_mermaid(self) -> Tuple[str, Tuple[Tuple[int, int]]]:
        main_style, str_main_links = self.get_nodes_mermaid(self.mainNodes)
        nodes_style, str_nodes_links = self.get_nodes_mermaid(self.nodes)

        topics_str = list_ros_element_2_list_str(self.topics.values())
        services_str = list_ros_element_2_list_str(self.services.values())
        actions_str = list_ros_element_2_list_str(self.actions.values())
        topics_style = "\n".join(topics_str)
        services_style = "\n".join(services_str)
        actions_style = "\n".join(actions_str)

        all_links = {
            name: str_main_links[name] + str_nodes_links[name] for name in LinkType
        }

        num_topic_publisher_links = [0, len(all_links[LinkType.TOPIC_PUBLISHER])]
        num_topic_subscriber_links = [
            num_topic_publisher_links[1],
            len(all_links[LinkType.TOPIC_SUBSCRIBER]) + num_topic_publisher_links[1],
        ]
        num_service_server_links = [
            num_topic_subscriber_links[1],
            len(all_links[LinkType.SERVICE_SERVER]) + num_topic_subscriber_links[1],
        ]
        num_service_client_links = [
            num_service_server_links[1],
            len(all_links[LinkType.SERVICE_CLIENT]) + num_service_server_links[1],
        ]
        num_action_server_links = [
            num_service_client_links[1],
            len(all_links[LinkType.ACTION_SERVER]) + num_service_client_links[1],
        ]
        num_action_client_links = [
            num_action_server_links[1],
            len(all_links[LinkType.ACTION_CLIENT]) + num_action_server_links[1],
        ]

        links_ranges = {
            LinkType.TOPIC_PUBLISHER: num_topic_publisher_links,
            LinkType.TOPIC_SUBSCRIBER: num_topic_subscriber_links,
            LinkType.SERVICE_SERVER: num_service_server_links,
            LinkType.SERVICE_CLIENT: num_service_client_links,
            LinkType.ACTION_SERVER: num_action_server_links,
            LinkType.ACTION_CLIENT: num_action_client_links,
        }
        print(main_style)
        print(nodes_style)
        mermaid_graph = [
            main_style,
            nodes_style,
            topics_style,
            services_style,
            actions_style,
            "\n".join(all_links[LinkType.TOPIC_PUBLISHER]),
            "\n".join(all_links[LinkType.TOPIC_SUBSCRIBER]),
            "\n".join(all_links[LinkType.SERVICE_SERVER]),
            "\n".join(all_links[LinkType.SERVICE_CLIENT]),
            "\n".join(all_links[LinkType.ACTION_SERVER]),
            "\n".join(all_links[LinkType.ACTION_CLIENT]),
        ]
        mermaid_str = "\n".join(mermaid_graph)
        return mermaid_str, links_ranges


def is_not_in_blacklist(element: Tuple[str, str], blacklist: List[str]) -> bool:
    """! Return true if the element name do not contains a fragment in the balcklist
    @param element (name, type) tuple
    @param exclude not desired elemts names list
    """
    for e in blacklist:
        if e in element[0]:
            return False
    return True

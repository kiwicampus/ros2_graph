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

import subprocess
from functools import partial
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node

ElementNameTypes = Tuple[str, List[str]]
RelatedNodes = Dict[str, List[str]]
ElementRelatedNodes = Dict[str, RelatedNodes]

rclpy.init()
dummy = Node("Graph_generator")


def between_patterns(pattern: Tuple[str], file: str) -> str:
    """!
    Return sed instruction to get text between two patters
    @param pattern (tuple) size two tuple with both patterns
    @param file (str) File to filter

    @return str bash instruction to filter a text file
    """
    instruction = ("sed -n '/%s/, /%s/{ /%s/! { /%s/! p } }' %s") % (
        pattern[0],
        pattern[1],
        pattern[0],
        pattern[1],
        file,
    )
    return instruction


def after_pattern(pattern: str, file: str) -> str:
    """!
    Return sed instruction to get text after a pattern
    @param pattern (str) pattern to filter
    @param file (str) File to filter

    @return str bash instruction to filter a text file
    """
    instruction = f"sed -n '/{pattern}/,$p' {file} | sed '1d'"
    return instruction


def get_clean_lines(query_instruction: str) -> List[str]:
    """! Perform a shell instruction and return the answer as a list of string

    @param query_instruction (str) ros2 cli query plus sed or grep filter instruction

    @return list lines
    """
    line_list = (
        subprocess.check_output(query_instruction, shell=True)
        .decode("utf-8")
        .splitlines()
    )

    return list(filter(lambda x: x != "", line_list))


def get_node_info_block(pattern: tuple, file_name: str) -> Tuple[Tuple[str, str]]:
    """! Get the name and the type of the ros elements in an ros2 node info block
    @param pattern: tuple of strings, marks that indicate the start and the end of that block
    @param file_name: File where is stored the ros node info response
    @return tuples (element, type)
    """
    filter_rule = (
        between_patterns(pattern, file_name)
        if len(pattern) > 1
        else after_pattern(pattern[0], file_name)
    )
    filter_rule += " | tr -d '[:blank:]'"
    return tuple(x.split(":") for x in get_clean_lines(filter_rule))


def join_name_and_namespace(name: str, namespace: str = None) -> str:
    """!  Put togheter name and name space on a sigle string
    @param name
    @param namespace: ignored for default namespace '/'
    @return str namespace/name
    """
    if namespace is None:
        return name
    if len(namespace) > 1:
        namespace += "/"
    return namespace + name


def is_not_in_blacklist(element: Tuple[str, str], blacklist: List[str]) -> bool:
    """! Return true if the element name do not contains a fragment in the balcklist
    @param element (name, type) tuple
    @param exclude not desired elemts names list
    """
    for e in blacklist:
        if e in element[0]:
            return False
    return True


def filter_topics(topic: Tuple[str, str]) -> bool:
    """! To filter undesired topics
    @param topic tuple(name, type)
    @return False if the topic is not desired
    """
    exclude = (
        "/parameter_events",
        "/rosout",
        "/tf",
        "/cascade_lifecycle_activations",
        "/cascade_lifecycle_states",
        "",
    )

    blacklist = ("/transition_event", "/_action")

    if topic[0] in exclude:
        return False

    return is_not_in_blacklist(topic, blacklist)


def get_topics_related_nodes(
    topics: List[ElementNameTypes], subscribers: bool
) -> ElementRelatedNodes:
    """!
    Get publishers or subcribers nodes for the specified topics

    @param topics (list[tuple(name, types list)]) List of topics with names and type
    @param publishers (bool) get publishers or subscribers nodes

    @return dict {topic_name: {type: str, nodes: [nodes_names]}}
    """
    topics_and_nodes = {
        topic[0]: {"type": "<br>".join(topic[1]), "nodes": []}
        for topic in filter(filter_topics, topics)
    }
    topics_names = topics_and_nodes.keys()

    get_endpoints_function = (
        dummy.get_publishers_info_by_topic
        if subscribers
        else dummy.get_subscriptions_info_by_topic
    )

    for topic in topics_names:
        endpoints = get_endpoints_function(topic)
        topics_and_nodes[topic]["nodes"] = [
            join_name_and_namespace(endpoint.node_name, endpoint.node_namespace)
            for endpoint in endpoints
        ]

    return topics_and_nodes


def get_services_related_nodes(
    services: List[ElementNameTypes], clients: bool
) -> ElementRelatedNodes:
    """! Get servers or clients nodes for the specified services
    @param services list of tuple(name: str, types: list)
    @param clients, True for get clients names, false for server names
    @return dictonary with structure {service_name: {"type": type, "nodes": client or server nodes names list}}
    """

    blacklist = (
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
    )
    filter_services = partial(is_not_in_blacklist, blacklist=blacklist)
    services_and_nodes = {
        service[0]: {"type": "<br>".join(service[1]), "nodes": []}
        for service in filter(filter_services, services)
    }

    services_names = services_and_nodes.keys()
    nodes_list = dummy.get_node_names_and_namespaces()

    get_services_function = (
        dummy.get_client_names_and_types_by_node
        if clients
        else dummy.get_service_names_and_types_by_node
    )

    for node in nodes_list:

        services = tuple(map(lambda x: x[0], get_services_function(*node)))
        filtered_services = tuple(
            filter((lambda service: service in services_names), services)
        )

        for service in filtered_services:
            services_and_nodes[service]["nodes"].append(join_name_and_namespace(*node))

    return services_and_nodes


def get_actions_related_nodes(
    actions: Tuple[ElementNameTypes], clients: bool
) -> ElementRelatedNodes:
    """!
    Get clients or servers nodes for the specified action

    @param actions  tuples of tuples(name, type)
    @param clients  get client or server nodes

    @return dict {action_name: [nodes_names]}
    """

    pattern = ("Action clients:", "Action servers:")
    aux_file = "actions.txt"
    filter_command = (
        between_patterns(pattern, aux_file)
        if clients
        else after_pattern(pattern[1], aux_file)
    ) + " | tr -d '[:blank:]'"

    actions_and_nodes = {
        action[0]: {"type": action[1], "nodes": []} for action in actions
    }
    actions_names = actions_and_nodes.keys()
    for action in actions_names:
        actions_and_nodes[action]["nodes"] = tuple(
            get_clean_lines(
                f"ros2 action info {action} >> {aux_file} && {filter_command} && rm {aux_file}"
            )
        )

    return actions_and_nodes


def mermaid_topics(
    node: str, topics: ElementRelatedNodes, subscribers: bool
) -> Tuple[str, int]:
    """! Construct the mermaid description to add topics to the graph
    @param node, main node name
    @param topics, {topic_name: {type: str, nodes: [nodes_names]}} dictionary
    @param subscribers, link subscribers or publishers nodes

    @return mermaid_topic_description list of lines with mermaid description for linked topics
    @return links_count, number of links
    """
    mermaid_topic_description = []
    links_count = 0
    for topic, topic_info in topics.items():
        n_publishers = len(topic_info["nodes"])
        links_count += 1 + n_publishers

        style = "topic" if n_publishers else "bugged"
        topic_node = f"{topic}([{topic}<br>{topic_info['type']}]):::{style}"

        if subscribers:
            topic_node += f" --> {node}"
            mermaid_topic_description.append(topic_node)
            mermaid_topic_description.extend(
                [f"{node}:::node --> {topic}" for node in topic_info["nodes"]]
            )

        else:
            topic_node = f"{node} --> " + topic_node
            mermaid_topic_description.append(topic_node)
            mermaid_topic_description.extend(
                [f"{topic} --> {node}:::node" for node in topic_info["nodes"]]
            )

    return mermaid_topic_description, links_count


def mermaid_services(
    node: str, services: ElementRelatedNodes, clients: bool
) -> Tuple[str, int]:
    """! Construct the mermaid description to add services to the graph
    @param node, main node name
    @param services, {service_name: {type: str, nodes: [nodes_names]}} dictionary
    @param clients, link clients or servers nodes

    @return mermaid_topic_description list of lines with mermaid description for linked services
    @return links_count, number of links
    """
    mermaid_service_description = []
    links_count = 0
    for service, service_info in services.items():
        n_clients = len(service_info["nodes"])
        links_count += 1 + n_clients

        style = "service" if n_clients else "bugged"
        service_node = f"{service}[/{service}<br>{service_info['type']}\]:::{style}"

        if clients:
            service_node = f"{node} o-.-o " + service_node
            mermaid_service_description.append(service_node)
            mermaid_service_description.extend(
                [f"{service} <-.-> {node}:::node" for node in service_info["nodes"]]
            )
        else:
            service_node = service_node + f" <-.-> {node}"
            mermaid_service_description.append(service_node)
            mermaid_service_description.extend(
                [f"{node}:::node  o-.-o {service}" for node in service_info["nodes"]]
            )

    return mermaid_service_description, links_count


def mermaid_actions(
    node: str, actions: ElementRelatedNodes, clients: bool
) -> Tuple[str, int]:
    """! Construct the mermaid description to add actions to the graph
    @param node, main node name
    @param actions, {action_name: {type: str, nodes: [nodes_names]}} dictionary
    @param subscribers, link clients or servers nodes

    @return mermaid_topic_description list of lines with mermaid description for linked actions
    @return links_count, number of links
    """
    mermaid_action_description = []
    links_count = 0
    for action, action_info in actions.items():

        n_clients = len(action_info["nodes"])
        links_count += 1 + n_clients

        style = "action" if n_clients else "bugged"

        action_node = (
            action + "{{" + action + "<br>" + action_info["type"] + "}}:::" + style
        )

        if clients:
            action_node = f"{node} <==> " + action_node
            mermaid_action_description.append(action_node)
            mermaid_action_description.extend(
                [f"{action} o==o {node}:::node" for node in action_info["nodes"]]
            )
        else:
            action_node += f" o==o {node}"
            mermaid_action_description.append(action_node)
            mermaid_action_description.extend(
                [f"{node}:::node <==> {action}" for node in action_info["nodes"]]
            )
    return mermaid_action_description, links_count


def get_node_graph(node, links_count):

    patterns = {
        "action_servers": ("Action Servers:", "Action Clients:"),
        "action_client": ("Action Clients:",),
    }

    subprocess.check_output(f"ros2 node info {node} >> node_info.txt", shell=True)
    elements = {
        k: get_node_info_block(pattern, file_name="node_info.txt")
        for k, pattern in patterns.items()
    }
    subprocess.check_output("rm node_info.txt", shell=True)
    action_clients = get_actions_related_nodes(elements["action_servers"], clients=True)
    action_servers = get_actions_related_nodes(elements["action_client"], clients=False)

    namespace_name = node.split("/")
    name = namespace_name[-1]
    namespace = "/".join(namespace_name[:-1])
    if namespace == "":
        namespace = "/"
    name_and_namespace = (name, namespace)

    subscribers = dummy.get_subscriber_names_and_types_by_node(*name_and_namespace)
    publishers = dummy.get_publisher_names_and_types_by_node(*name_and_namespace)
    services_server = dummy.get_service_names_and_types_by_node(*name_and_namespace)
    services_client = dummy.get_client_names_and_types_by_node(*name_and_namespace)

    topics_subscribers = get_topics_related_nodes(subscribers, subscribers=True)
    topics_publishers = get_topics_related_nodes(publishers, subscribers=False)
    service_clients = get_services_related_nodes(services_server, clients=True)
    service_servers = get_services_related_nodes(services_client, clients=False)

    mermaid_graph_description, links_count_subs = mermaid_topics(
        node, topics_subscribers, subscribers=True
    )
    links_count += links_count_subs
    mermaid_list, links_count_pubs = mermaid_topics(
        node, topics_publishers, subscribers=False
    )
    mermaid_graph_description.extend(mermaid_list)
    links_count += links_count_pubs

    mermaid_list, links_count_sclients = mermaid_services(
        node, service_clients, clients=True
    )
    mermaid_graph_description.extend(mermaid_list)
    links_count += links_count_sclients

    mermaid_list, links_count_sserver = mermaid_services(
        node, service_servers, clients=False
    )
    mermaid_graph_description.extend(mermaid_list)
    links_count += links_count_sserver

    start_action_links = links_count
    mermaid_list, links_count_aclients = mermaid_actions(
        node, action_clients, clients=False
    )
    mermaid_graph_description.extend(mermaid_list)
    links_count += links_count_aclients
    mermaid_list, links_count_aserver = mermaid_actions(
        node, action_servers, clients=True
    )
    mermaid_graph_description.extend(mermaid_list)
    links_count += links_count_sserver

    action_links = list(range(start_action_links, links_count))

    return mermaid_graph_description, action_links, links_count

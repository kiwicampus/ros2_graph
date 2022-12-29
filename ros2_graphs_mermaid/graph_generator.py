import subprocess
from functools import reduce, partial
import rclpy
from rclpy.node import Node


rclpy.init()
dummy = Node("Graph_generator")


def between_patterns(pattern: tuple, file: str) -> str:
    """!
    Return sed instruction to get text between two patters
    @param pattern (tuple) size tow tuple with both patterns
    @param file (str) File to filter

    @return str filtered text
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

    @return str filterd text
    """
    instruction = f"sed -n '/{pattern}/,$p' {file} | sed '1d'"
    return instruction


def get_clean_lines(query_instruction: str) -> list:
    """! Extrac nodes names from ros cli query

    @param query_instruction (str) ros2 cli query plus sed or grep filter instruction

    @return list lines
    """
    line_list = (
        subprocess.check_output(
            query_instruction,
            shell=True,
        )
        .decode("utf-8")
        .splitlines()
    )

    return filter(lambda x: x != "", line_list)


def get_node_info_block(pattern: tuple):
    """! Extract info from an specific block of ros2 node info instruction
    @param pattern: tuple of strings, marks taht indicate the start and the end of that block
    @return tuples (element, type)
    """
    file_name = "node_info.txt"
    filter_rule = (
        between_patterns(pattern, file_name)
        if len(pattern) > 1
        else after_pattern(pattern[0], file_name)
    )
    filter_rule += " | tr -d '[:blank:]'"
    return tuple(x.split(":") for x in get_clean_lines(filter_rule))


def join_name_and_namespace(name: str, namespace: str = None):
    if namespace is None:
        return name
    if len(namespace) > 1:
        namespace += "/"
    return namespace + name


def is_not_exclude(service, exclude):
    for e in exclude:
        if e in service[0]:
            return False
    return True


def get_topic_info(topics: list, publishers: bool) -> dict:
    """!
    Get publishers or subcribers nodes from nodes names

    @param topics (list[tuples]) List of topics with names and type
    @param publishers (bool) get publishers or subscribers nodes

    @return tuples (topic_name, [nodes_names])
    """

    def filter_topics(topic):
        exclude = (
            "/parameter_events",
            "/rosout",
            "/tf",
            "/cascade_lifecycle_activations",
            "/cascade_lifecycle_states",
            "",
        )

        exclude_fracment = ("/transition_event", "/_action")

        if topic[0] in exclude:
            return False

        return is_not_exclude(topic, exclude_fracment)

    topics_and_nodes = {
        topic[0]: {"type": topic[1][0], "nodes": []}
        for topic in filter(filter_topics, topics)
    }
    topics_names = topics_and_nodes.keys()

    get_endpoints_function = (
        dummy.get_publishers_info_by_topic
        if publishers
        else dummy.get_subscriptions_info_by_topic
    )

    for topic in topics_names:
        endpoints = get_endpoints_function(topic)
        topics_and_nodes[topic]["nodes"] = [
            join_name_and_namespace(endpoint.node_name, endpoint.node_namespace)
            for endpoint in endpoints
        ]

    return topics_and_nodes


def get_server_info(services: map, clients: bool):
    """! Filter avalible services
    @param services (map) _description_
    """

    exclude = (
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
    filter_services = partial(is_not_exclude, exclude=exclude)
    services_and_nodes = {
        service[0]: {"type": service[1][0], "nodes": []}
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


def get_action_info(actions: map, clients: bool):
    """!
    Get publishers or subcribers nodes from nodes names

    @param raw_topic_list (map) tuples actions and types
    @param subscribers (bool) get subscribers or publishers nodes

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
                f"ros2 action info {action} >> {aux_file} && {filter_command} && rm {aux_file}",
            )
        )

    return actions_and_nodes


def mermaid_topics(node: str, topics: dict, subscribers: bool, links_count: int):
    mermaid_topic_description = []
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
    node: str, service_dictionary: dict, clients: bool, links_count: int
):
    mermaid_service_description = []
    for service, service_info in service_dictionary.items():
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
    node: str, action_dictionary: dict, clients: bool, links_count: int
):
    mermaid_action_description = []
    for action, action_info in action_dictionary.items():

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
                [f"{action} o==o {node}:::node" for node in action_info["nodes"]]
            )
    return mermaid_action_description, links_count


def get_node_graph(node, links_count):

    patterns = {
        "action_servers": ("Action Servers:", "Action Clients:"),
        "action_client": ("Action Clients:",),
    }

    subprocess.check_output(
        f"ros2 node info {node} >> node_info.txt",
        shell=True,
    )
    elements = {k: get_node_info_block(pattern) for k, pattern in patterns.items()}
    subprocess.check_output(
        "rm node_info.txt",
        shell=True,
    )
    action_clients = get_action_info(elements["action_servers"], True)
    action_servers = get_action_info(elements["action_client"], False)

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

    topics_subscribers = get_topic_info(subscribers, True)
    topics_publishers = get_topic_info(publishers, False)
    service_clients = get_server_info(services_server, True)
    service_servers = get_server_info(services_client, False)

    mermaid_graph_description, links_count = mermaid_topics(
        node, topics_subscribers, subscribers=True, links_count=links_count
    )
    mermaid_list, links_count = mermaid_topics(
        node, topics_publishers, subscribers=False, links_count=links_count
    )
    mermaid_graph_description.extend(mermaid_list)

    mermaid_list, links_count = mermaid_services(
        node, service_clients, clients=True, links_count=links_count
    )
    mermaid_graph_description.extend(mermaid_list)
    mermaid_list, links_count = mermaid_services(
        node, service_servers, clients=False, links_count=links_count
    )
    mermaid_graph_description.extend(mermaid_list)

    start_action_links = links_count
    mermaid_list, links_count = mermaid_actions(
        node, action_clients, clients=False, links_count=links_count
    )
    mermaid_graph_description.extend(mermaid_list)
    mermaid_list, links_count = mermaid_actions(
        node, action_servers, clients=True, links_count=links_count
    )
    mermaid_graph_description.extend(mermaid_list)

    action_links = list(range(start_action_links, links_count))

    return mermaid_graph_description, action_links, links_count


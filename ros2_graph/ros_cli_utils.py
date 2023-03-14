import subprocess
from typing import List, Tuple
import os


def remove_file(file: str) -> None:
    """!
    Just remove a file if ist exist
    """
    if os.path.exists(file):
        os.remove(file)


def save_ros_node_info(node_name: str, file: str) -> None:
    """!
    Save ros node informatin in a text file
    """
    remove_file(file)
    subprocess.check_output(f"ros2 node info {node_name} >> {file}", shell=True)


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


def split_full_name(full_name: str) -> Tuple(str, str):
    """!
    split a ros name into name and namespace
    """
    namespace, name = full_name.rsplit("/", 1)
    if len(namespace) < 1:
        namespace = "/"
    return name, namespace


def get_name_namespace_type(line: str) -> Tuple[str]:
    """! split name, namespace and type from a line of a ros cli response
    @param line: a line of the ros2 cli response. Format: namespace/name:type
    """
    full_name, type = line.split(":")
    name, namespace = split_full_name(full_name)
    name = "/" + name
    return name, namespace, type


def get_node_info_block(pattern: tuple, file: str) -> Tuple[Tuple[str]]:
    """! Get the name and the type of the ros elements in an ros2 node info block
    @param pattern: tuple of strings, marks that indicate the start and the end of that block
    @param file_name: File where is stored the ros node info response
    @return tuples (element_name, element_namespace, ros_type)
    """
    filter_rule = (
        between_patterns(pattern, file)
        if len(pattern) > 1
        else after_pattern(pattern[0], file)
    )
    filter_rule += " | tr -d '[:blank:]'"
    elements = tuple(map(get_name_namespace_type, get_clean_lines(filter_rule)))
    return elements


def get_action_related_nodes(action_name: str, clients: bool) -> Tuple[Tuple[str]]:
    """!
    Get clients or servers nodes for the specified action

    @param actions  tuples of tuples(name, name_space, type)
    @param clients  get client or server nodes

    @return tuple of tuples (name, namespace)
    """

    pattern = ("Action clients:", "Action servers:")
    aux_file = "actions.txt"
    filter_command = (
        between_patterns(pattern, aux_file)
        if clients
        else after_pattern(pattern[1], aux_file)
    ) + " | tr -d '[:blank:]'"

    raw_nodes_names = get_clean_lines(
        f"ros2 action info {action_name} >> {aux_file} && {filter_command} && rm {aux_file}"
    )
    nodes_names = map(split_full_name, raw_nodes_names)

    return nodes_names

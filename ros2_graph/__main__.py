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

import argparse
import subprocess
from os.path import splitext
from os import remove
from typing import Dict
import yaml

from .graph_generator import GraphGenerator
from .rosElement import ELEMENT_TYPE, LINK_TYPE


def fromDictOrDefault(dictionary: Dict[str, str], key: str, default: str):
    return dictionary[key] if key in dictionary else default


def get_style(style_file: str):
    with open(style_file) as f:
        yaml_data = f.read()
    style_dict = yaml.load(yaml_data)

    shapes = fromDictOrDefault(style_dict, "shapes", {})
    colors = fromDictOrDefault(style_dict, "colors", {})
    links_display = fromDictOrDefault(style_dict, "links_display", {})
    links_style = fromDictOrDefault(style_dict, "links_style", {})
    display_keys = fromDictOrDefault(style_dict, "display_keys", True)

    shapesDict = {
        ELEMENT_TYPE.MAIN: fromDictOrDefault(shapes, "main", ["[", "]"]),
        ELEMENT_TYPE.NODE: fromDictOrDefault(shapes, "node", ["[", "]"]),
        ELEMENT_TYPE.TOPIC: fromDictOrDefault(shapes, "topic", ["([", "])"]),
        ELEMENT_TYPE.SERVICE: fromDictOrDefault(shapes, "service", ["[/", "\\]"]),
        ELEMENT_TYPE.ACTION: fromDictOrDefault(shapes, "action", ["{{", "}}"]),
    }
    colorsDict = {
        ELEMENT_TYPE.MAIN: fromDictOrDefault(
            colors,
            "main",
            "opacity:0.9,fill:#059,stroke:#09F,stroke-width:4px,color:#fff",
        ),
        ELEMENT_TYPE.NODE: fromDictOrDefault(
            colors,
            "node",
            "opacity:0.9,fill:#2A0,stroke:#391,stroke-width:4px,color:#fff",
        ),
        ELEMENT_TYPE.TOPIC: fromDictOrDefault(
            colors,
            "topic",
            "opacity:0.9,fill:#852,stroke:#CCC,stroke-width:2px,color:#fff",
        ),
        ELEMENT_TYPE.SERVICE: fromDictOrDefault(
            colors,
            "service",
            "opacity:0.9,fill:#3B8062,stroke:#3B6062,stroke-width:2px,color:#fff",
        ),
        ELEMENT_TYPE.ACTION: fromDictOrDefault(
            colors,
            "action",
            "opacity:0.9,fill:#66A,stroke:#225,stroke-width:2px,color:#fff",
        ),
    }
    no_conected = fromDictOrDefault(
        colors,
        "no_conected",
        "opacity:0.9,fill:#933,stroke:#800,stroke-width:2px,color:#fff",
    )

    linkStrs = {
        LINK_TYPE.TOPIC_PUBLISHER: fromDictOrDefault(
            links_display, "topics_publisher", "-->"
        ),
        LINK_TYPE.TOPIC_SUBSCRIBER: fromDictOrDefault(
            links_display, "topics_subscriber", "-->"
        ),
        LINK_TYPE.SERVICE_SERVER: fromDictOrDefault(
            links_display, "services_server", "o-.-o"
        ),
        LINK_TYPE.SERVICE_CLIENT: fromDictOrDefault(
            links_display, "services_client", "<-.->"
        ),
        LINK_TYPE.ACTION_SERVER: fromDictOrDefault(
            links_display, "action_server", "o==o"
        ),
        LINK_TYPE.ACTION_CLIENT: fromDictOrDefault(
            links_display, "action_client", "<==>"
        ),
    }
    linkStyle = {
        LINK_TYPE.TOPIC_PUBLISHER: fromDictOrDefault(
            links_style, "topics_publisher", None
        ),
        LINK_TYPE.TOPIC_SUBSCRIBER: fromDictOrDefault(
            links_style, "topics_subscriber", None
        ),
        LINK_TYPE.SERVICE_SERVER: fromDictOrDefault(
            links_style, "services_server", None
        ),
        LINK_TYPE.SERVICE_CLIENT: fromDictOrDefault(
            links_style, "services_client", None
        ),
        LINK_TYPE.ACTION_SERVER: fromDictOrDefault(
            links_style, "action_server", "color:green;"
        ),
        LINK_TYPE.ACTION_CLIENT: fromDictOrDefault(
            links_style, "action_client", "color:green;"
        ),
    }

    style_settings = {
        "shapes": shapesDict,
        "colors": colorsDict,
        "no_conected": no_conected,
        "links_str": linkStrs,
        "links_style": linkStyle,
        "display_keys": display_keys,
    }

    return style_settings


def main():
    parser = argparse.ArgumentParser(
        description="Create mermaid graphs from your ros2 nodes"
    )
    parser.add_argument(
        "nodes", metavar="/node", type=str, nargs="+", help="main nodes of your graph"
    )
    parser.add_argument(
        "-o",
        "--out_file",
        dest="out_file",
        help="set output file otherwise the graph will be printed on the console",
        default="None",
        type=str,
    )
    parser.add_argument(
        "--outputFormat",
        dest="out_type",
        help="set an output format",
        choices=("console", "md", "svg", "png", "pdf"),
        default="console",
        type=str,
    )
    parser.add_argument(
        "--styleConfig",
        dest="style_config",
        help="Pass a yaml file for style configuration",
        default=None,
        type=str,
    )
    args = parser.parse_args()

    nodes = args.nodes
    out_file, aux_out_type = splitext(args.out_file)
    out_type = "console"
    style_config = get_style(args.style_config)

    # use file extension as default type
    if aux_out_type != "":
        out_type = aux_out_type

    if args.out_type != "console":
        out_type = "." + args.out_type

    # use .md as output type if is not declared but there is an output file
    if out_file != "None" and out_type == "console":
        out_type = ".md"

    if out_type != "console" and out_file == "None":
        raise Exception("Output file is missing")

    graph_generator = GraphGenerator(style_config)

    for node in nodes:
        graph_generator.get_node_graph(node)

    mermaid_graph, links_ranges = graph_generator.get_mermaid()

    links_str = style_config["links_str"]
    shapes = style_config["shapes"]
    mermaid_convention = (
        "\n".join(
            [
                "subgraph keys[<b>Keys<b/>]",
                "subgraph nodes[<b><b/>]",
                "topicb((No connected)):::bugged",
                f"main_node{shapes[ELEMENT_TYPE.MAIN][0]}main{shapes[ELEMENT_TYPE.MAIN][1]}:::main",
                "end",
                "subgraph connection[<b><b/>]",
                f"node1{shapes[ELEMENT_TYPE.NODE][0]}node1{shapes[ELEMENT_TYPE.NODE][1]}:::node",
                f"node2{shapes[ELEMENT_TYPE.NODE][0]}node2{shapes[ELEMENT_TYPE.NODE][1]}:::node",
                f"node1 {links_str[LINK_TYPE.SERVICE_SERVER]}|to server| service{shapes[ELEMENT_TYPE.SERVICE][0]}Service<br>service/Type{shapes[ELEMENT_TYPE.SERVICE][1]}:::service",
                f"service {links_str[LINK_TYPE.SERVICE_CLIENT]}|to client| node2",
                f"node1 {links_str[LINK_TYPE.TOPIC_PUBLISHER]}|publish| topic{shapes[ELEMENT_TYPE.TOPIC][0]}Topic<br>topic/Type{shapes[ELEMENT_TYPE.TOPIC][1]}:::topic",
                f"topic {links_str[LINK_TYPE.TOPIC_SUBSCRIBER]}|subscribe| node2",
                f"node1 {links_str[LINK_TYPE.ACTION_SERVER]}|to server| action{shapes[ELEMENT_TYPE.ACTION][0]}/Action<br>action/Type/{shapes[ELEMENT_TYPE.ACTION][1]}:::action",
                f"action {links_str[LINK_TYPE.ACTION_CLIENT]}|to client| node2",
                "end",
                "end",
            ]
        )
        if style_config["display_keys"]
        else ""
    )

    # Add action links of conventions sub graph (the 4th and the 5th)
    # action_links.extend([links_count + 4, links_count + 5])
    # action_links_style = (
    #    "linkStyle " + ",".join(map(str, action_links)) + " fill:none,stroke:green;"
    # )

    colors = style_config["colors"]
    mermaid_style = [
        "classDef node " + colors[ELEMENT_TYPE.NODE],
        "classDef action " + colors[ELEMENT_TYPE.ACTION],
        "classDef service " + colors[ELEMENT_TYPE.SERVICE],
        "classDef topic " + colors[ELEMENT_TYPE.TOPIC],
        "classDef main " + colors[ELEMENT_TYPE.MAIN],
        "classDef bugged " + style_config["no_conected"],
    ]

    if style_config["display_keys"]:
        mermaid_style += [
            "style keys opacity:0.15,fill:#FFF",
            "style nodes opacity:0.15,fill:#FFF",
            "style connection opacity:0.15,fill:#FFF",
        ]

    mermaid_style = "\n".join(mermaid_style)

    heading = "```mermaid\nflowchart LR\n"

    mermaid_graph = "\n".join(
        [
            heading,
            mermaid_graph,
            mermaid_convention,
            mermaid_style,
            "```\n",
        ]
    )

    # print on console
    if out_file == "None":
        print(mermaid_graph)
        return

    with open(out_file + ".md", "a") as file:
        file.write(mermaid_graph)

    # Save .md file
    if out_type == ".md":
        return

    # Export image
    try:
        command = (
            "npx -p @mermaid-js/mermaid-cli mmdc -i "
            + out_file
            + ".md -o "
            + out_file
            + out_type
            + " -b transparent"
        )
        subprocess.run(command, shell=True)
    except:
        print(
            "An error using mermaid-cli look for more details in https://github.com/mermaid-js/mermaid-cli"
        )
    finally:
        remove(out_file + ".md")


if __name__ == "__main__":
    main()

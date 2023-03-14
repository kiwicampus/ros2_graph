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
from .ros_element import ElementType, LinkType


def get_style(style_file: str):
    with open(style_file) as f:
        yaml_data = f.read()
    style_dict = yaml.load(yaml_data)

    shapes = style_dict.get("shapes", {})
    colors = style_dict.get("colors", {})
    links_display = style_dict.get("links_display", {})
    links_style = style_dict.get("links_style", {})
    display_keys = style_dict.get("display_keys", True)

    shapesDict = {
        ElementType.MAIN: shapes.get("main", ["[", "]"]),
        ElementType.NODE: shapes.get("node", ["[", "]"]),
        ElementType.TOPIC: shapes.get("topic", ["([", "])"]),
        ElementType.SERVICE: shapes.get("service", ["[/", "\\]"]),
        ElementType.ACTION: shapes.get("action", ["{{", "}}"]),
    }
    colorsDict = {
        ElementType.MAIN: colors.get(
            "main", "opacity:0.9,fill:#059,stroke:#09F,stroke-width:4px,color:#fff"
        ),
        ElementType.NODE: colors.get(
            "node", "opacity:0.9,fill:#2A0,stroke:#391,stroke-width:4px,color:#fff"
        ),
        ElementType.TOPIC: colors.get(
            "topic", "opacity:0.9,fill:#852,stroke:#CCC,stroke-width:2px,color:#fff"
        ),
        ElementType.SERVICE: colors.get(
            "service",
            "opacity:0.9,fill:#3B8062,stroke:#3B6062,stroke-width:2px,color:#fff",
        ),
        ElementType.ACTION: colors.get(
            "action", "opacity:0.9,fill:#66A,stroke:#225,stroke-width:2px,color:#fff"
        ),
    }
    no_conected = colors.get(
        "no_conected", "opacity:0.9,fill:#933,stroke:#800,stroke-width:2px,color:#fff"
    )

    linkStrs = {
        LinkType.TOPIC_PUBLISHER: links_display.get("topics_publisher", "-->"),
        LinkType.TOPIC_SUBSCRIBER: links_display.get("topics_subscriber", "-->"),
        LinkType.SERVICE_SERVER: links_display.get("services_server", "o-.-o"),
        LinkType.SERVICE_CLIENT: links_display.get("services_client", "<-.->"),
        LinkType.ACTION_SERVER: links_display.get("action_server", "o==o"),
        LinkType.ACTION_CLIENT: links_display.get("action_client", "<==>"),
    }
    linkStyle = {
        LinkType.TOPIC_PUBLISHER: links_style.get("topics_publisher", None),
        LinkType.TOPIC_SUBSCRIBER: links_style.get("topics_subscriber", None),
        LinkType.SERVICE_SERVER: links_style.get("services_server", None),
        LinkType.SERVICE_CLIENT: links_style.get("services_client", None),
        LinkType.ACTION_SERVER: links_style.get("action_server", "color:green;"),
        LinkType.ACTION_CLIENT: links_style.get("action_client", "color:green;"),
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
                f"main_node{shapes[ElementType.MAIN][0]}main{shapes[ElementType.MAIN][1]}:::main",
                "end",
                "subgraph connection[<b><b/>]",
                f"node1{shapes[ElementType.NODE][0]}node1{shapes[ElementType.NODE][1]}:::node",
                f"node2{shapes[ElementType.NODE][0]}node2{shapes[ElementType.NODE][1]}:::node",
                f"node1 {links_str[LinkType.SERVICE_SERVER]}|to server| service{shapes[ElementType.SERVICE][0]}Service<br>service/Type{shapes[ElementType.SERVICE][1]}:::service",
                f"service {links_str[LinkType.SERVICE_CLIENT]}|to client| node2",
                f"node1 {links_str[LinkType.TOPIC_PUBLISHER]}|publish| topic{shapes[ElementType.TOPIC][0]}Topic<br>topic/Type{shapes[ElementType.TOPIC][1]}:::topic",
                f"topic {links_str[LinkType.TOPIC_SUBSCRIBER]}|subscribe| node2",
                f"node1 {links_str[LinkType.ACTION_SERVER]}|to server| action{shapes[ElementType.ACTION][0]}/Action<br>action/Type/{shapes[ElementType.ACTION][1]}:::action",
                f"action {links_str[LinkType.ACTION_CLIENT]}|to client| node2",
                "end",
                "end",
            ]
        )
        if style_config["display_keys"]
        else ""
    )

    links_numbers = {
        link_type: list(range(*ranges))
        for link_type, ranges in links_ranges.items()
        if ranges[1] - ranges[0] > 0
    }

    # Add keys box links
    if style_config["display_keys"]:
        last_link = links_ranges[LinkType.ACTION_CLIENT][1]
        if LinkType.TOPIC_PUBLISHER in links_numbers:
            links_numbers[LinkType.TOPIC_PUBLISHER].append(last_link + 2)
        if LinkType.TOPIC_SUBSCRIBER in links_numbers:
            links_numbers[LinkType.TOPIC_SUBSCRIBER].append(last_link + 3)
        if LinkType.SERVICE_SERVER in links_numbers:
            links_numbers[LinkType.SERVICE_SERVER].append(last_link)
        if LinkType.SERVICE_CLIENT in links_numbers:
            links_numbers[LinkType.SERVICE_CLIENT].append(last_link + 1)
        if LinkType.ACTION_SERVER in links_numbers:
            links_numbers[LinkType.ACTION_SERVER].append(last_link + 4)
        if LinkType.ACTION_CLIENT in links_numbers:
            links_numbers[LinkType.ACTION_CLIENT].append(last_link + 5)

    links_styles = "\n".join(
        [
            "linkStyle "
            + ",".join(map(str, numbers))
            + " "
            + style_config["links_style"][link_type]
            for link_type, numbers in links_numbers.items()
            if style_config["links_style"][link_type] != None
            and style_config["links_style"][link_type] != "None"
        ]
    )

    colors = style_config["colors"]
    mermaid_style = [
        "classDef node " + colors[ElementType.NODE],
        "classDef action " + colors[ElementType.ACTION],
        "classDef service " + colors[ElementType.SERVICE],
        "classDef topic " + colors[ElementType.TOPIC],
        "classDef main " + colors[ElementType.MAIN],
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
            links_styles,
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

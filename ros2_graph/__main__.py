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
from functools import reduce

from .graph_generator import get_node_graph


def main():
    # Init arg parser
    parser = argparse.ArgumentParser(
        description="Create mermaid graphs from your ros2 nodes"
    )
    # Create arg for nodes lidt
    parser.add_argument(
        "nodes", metavar="/node", type=str, nargs="+", help="main nodes of your graph"
    )
    # Create arg for file output
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
    args = parser.parse_args()

    # Copy parse into variables
    nodes = args.nodes
    out_file, aux_out_type = splitext(args.out_file)
    out_type = "console"

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

    nodes_description = []
    action_links = []
    links_count = 0
    # For every requested node, get the graph
    for node in nodes:
        mermaid, action_links_aux, links_count = get_node_graph(node, links_count)
        nodes_description += mermaid
        action_links += action_links_aux
    # remove duplicates
    no_duplicates = [
        i for n, i in enumerate(nodes_description) if i not in nodes_description[:n]
    ]
    links_count = links_count - (len(nodes_description) - len(no_duplicates))
    nodes_description = "\n".join(no_duplicates)

    # Add main nodes style
    main_nodes_style = reduce(lambda a, node: a + f"{node}:::main_node\n", nodes, "")

    # Add keys subgraph
    mermaid_convention = "\n".join(
        [
            "%% Keys",
            "subgraph keys[<b>Keys<b/>]",
            "subgraph nodes[<b><b/>]",
            "topicb((No connected)):::bugged",
            "main_node:::main_node",
            "end",
            "subgraph connection[<b><b/>]",
            "node1:::node",
            "node2:::node",
            "node1 o-. to server .-o service[/Service<br>service/Type\]:::service",
            "service <-. to client .-> node2",
            "node1 -- publish --> topic([Topic<br>topic/Type]):::topic",
            "topic -- subscribe --> node2",
            "node1 o== to server ==o action{{/Action<br>action/Type/}}:::action",
            "action <== to client ==> node2",
            "end",
            "end",
            "style keys opacity:0.15,fill:#FFF",
            "style nodes opacity:0.15,fill:#FFF",
            "style connection opacity:0.15,fill:#FFF\n",
        ]
    )
    # Add action links of conventions sub graph (the 4th and the 5th)
    keys_action_links = [links_count + 4, links_count + 5]
    mermaid_convention += "linkStyle " + ",".join(
        map(str, keys_action_links)) + " fill:none,stroke:green;\n"

    # Add class definition and styles
    mermaid_style = [
        "%% Styles definition",
        "classDef node opacity:0.9,fill:#2A0,stroke:#391,stroke-width:4px,color:#fff",
        "classDef action opacity:0.9,fill:#66A,stroke:#225,stroke-width:2px,color:#fff",
        "classDef service opacity:0.9,fill:#3B8062,stroke:#3B6062,stroke-width:2px,color:#fff",
        "classDef topic opacity:0.9,fill:#852,stroke:#CCC,stroke-width:2px,color:#fff",
        "classDef main_node opacity:0.9,fill:#059,stroke:#09F,stroke-width:4px,color:#fff",
        "classDef bugged opacity:0.9,fill:#933,stroke:#800,stroke-width:2px,color:#fff",
    ]
    # Add action links style
    action_links_style = (
        "linkStyle " + ",".join(map(str, action_links)) + " fill:none,stroke:green;"
    )
    if action_links.__len__() > 0:
        mermaid_style.append(action_links_style)
    # Turn list into str
    mermaid_style = "\n".join(mermaid_style)

    # Define mermaid flowchart header
    heading = "\n```mermaid\nflowchart LR"

    # Join texts for final result
    mermaid_graph = "\n".join(
        [
            heading,
            nodes_description,
            main_nodes_style,
            mermaid_convention,
            mermaid_style,
            "```\n",
        ]
    )

    # print on console
    if out_file == "None":
        print(mermaid_graph)
        return

    with open(out_file + ".md", "w") as file:
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

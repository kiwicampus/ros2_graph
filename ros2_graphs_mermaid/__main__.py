
import sys
from functools import reduce

from .graph_generator import get_node_graph

def main(nodes: list):
    heading = "\n```mermaid\nflowchart LR\n"

    if not isinstance(nodes, list):
        nodes = [nodes]

    nodes_description = []
    action_links = []
    links_count = 0
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
    main_nodes_style = reduce(lambda a, node: a + f"{node}:::main_node\n", nodes, "")

    mermaid_convention = [
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
    ]

    mermaid_style = [
        "classDef node opacity:0.9,fill:#2A0,stroke:#391,stroke-width:4px,color:#fff",
        "classDef action opacity:0.9,fill:#66A,stroke:#225,stroke-width:2px,color:#fff",
        "classDef service opacity:0.9,fill:#3B8062,stroke:#3B6062,stroke-width:2px,color:#fff",
        "classDef topic opacity:0.9,fill:#852,stroke:#CCC,stroke-width:2px,color:#fff",
        "classDef main_node opacity:0.9,fill:#059,stroke:#09F,stroke-width:4px,color:#fff",
        "classDef bugged opacity:0.9,fill:#933,stroke:#800,stroke-width:2px,color:#fff",
        "style keys opacity:0.15,fill:#FFF",
        "style nodes opacity:0.15,fill:#FFF",
        "style connection opacity:0.15,fill:#FFF",
    ]

    action_links.extend([links_count + 4, links_count + 5])
    action_links_style = (
        "linkStyle " + ",".join(map(str, action_links)) + " fill:none,stroke:green;"
    )
    mermaid_style.append(action_links_style)

    print(heading)
    print(nodes_description)
    print(main_nodes_style)
    print("\n".join(mermaid_convention))
    print("\n".join(mermaid_style))
    print("```\n")


if __name__ == "__main__":
    main(sys.argv[1:])
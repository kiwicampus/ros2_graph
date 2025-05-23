# ros2_graph
Generate Mermaid descriptions of ROS 2 graphs for your Markdown files.

## Motivation

To improve architectural documentation, visualizations of nodes and topics are needed. However, maintaining or manually creating these graphs is often tedious and error-prone. Therefore, an automated tool is necessary.

The first alternative is [rqt-graph](http://wiki.ros.org/rqt_graph), but it does not include services or actions and may include extraneous information when filtering by node. The typical workflow involves connecting to a terminal where the ROS system is running, launching `rqt-graph`, saving the resulting image, and uploading it to the cloud to include it in the documentation.

To simplify this process and include all relevant information, this tool was created.

## Installation

ROS 2 must be installed beforehand (tested on the Humble distribution).

```bash
pip install ros2-graph
```

To export the diagram as an image, install npm and [mermaid-cli](https://github.com/mermaid-js/mermaid-cli) on your computer:

```bash
curl -sL https://deb.nodesource.com/setup_16.x | sudo -E bash -
sudo apt install nodejs
npm install -g @mermaid-js/mermaid-cli
```

## How it works

Suppose you want to generate a graph that shows how a node relates to other nodes via topics, services, and actions. Ensure your node is running. Then, run the following command:

```bash
ros2_graph your_node
```

This will print a Mermaid graph description to the console. Copy and paste it into the node's README. 

To export the output to a file, use the `-o` flag:

```bash
ros2_graph /turtlesim -o turtle_diagram.md
```

You can also export the diagram as an image (PNG, SVG, or PDF) by specifying the desired file extension or using the `--outputFormat` flag:

```bash
ros2_graph /turtlesim -o turtle_diagram.png
ros2_graph /turtlesim -o turtle_diagram --outputFormat png
```

**Note:** Make sure to copy everything between ` ```mermaid ` and ` ``` `, including the markers themselves.

GitHub supports Mermaid graphs. You can also view them in VS Code by installing the "bierner.markdown-mermaid" extension in your `devcontainer.json`.

What about nodes that are closely related, such as range sensors? Don't worry—you can include as many nodes as you want:

```bash
ros2_graph node_1 node_2 … node_n
```

Example:

```bash
ros2_graph /turtlesim /teleop_turtle
```
See an example graph:

```mermaid
flowchart LR
/turtlesim:::main
/teleop_turtle:::node
/turtle1cmd_vel([/turtle1cmd_vel<br>geometry_msgs/msg/Twist]):::topic
/turtle1color_sensor([/turtle1color_sensor<br>turtlesim/msg/Color]):::bugged
/turtle1pose([/turtle1pose<br>turtlesim/msg/Pose]):::bugged
/clear[//clear<br>std_srvs/srv/Empty\]:::bugged
/kill[//kill<br>turtlesim/srv/Kill\]:::bugged
/reset[//reset<br>std_srvs/srv/Empty\]:::bugged
/spawn[//spawn<br>turtlesim/srv/Spawn\]:::bugged
/turtle1set_pen[//turtle1set_pen<br>turtlesim/srv/SetPen\]:::bugged
/turtle1teleport_absolute[//turtle1teleport_absolute<br>turtlesim/srv/TeleportAbsolute\]:::bugged
/turtle1teleport_relative[//turtle1teleport_relative<br>turtlesim/srv/TeleportRelative\]:::bugged
/turtle1/rotate_absolute{{/turtle1/rotate_absolute<br>turtlesim/action/RotateAbsolute}}:::action
/clear o-.-o /turtlesim
/kill o-.-o /turtlesim
/reset o-.-o /turtlesim
/spawn o-.-o /turtlesim
/turtle1set_pen o-.-o /turtlesim
/turtle1teleport_absolute o-.-o /turtlesim
/turtle1teleport_relative o-.-o /turtlesim
/teleop_turtle <==> /turtle1/rotate_absolute
/turtle1/rotate_absolute o==o /turtlesim
/turtle1cmd_vel --> /turtlesim
/turtlesim --> /turtle1color_sensor
/turtlesim --> /turtle1pose
/teleop_turtle --> /turtle1cmd_vel
subgraph keys[<b>Keys<b/>]
subgraph nodes[<b><b/>]
topicb((No connected)):::bugged
main_node:::main
end
subgraph connection[<b><b/>]
node1:::node
node2:::node
node1 o-. to server .-o service[/Service<br>service/Type\]:::service
service <-. to client .-> node2
node1 -- publish --> topic([Topic<br>topic/Type]):::topic
topic -- subscribe --> node2
node1 o== to server ==o action{{/Action<br>action/Type/}}:::action
action <== to client ==> node2
end
end
classDef node opacity:0.9,fill:#2A0,stroke:#391,stroke-width:4px,color:#fff
classDef action opacity:0.9,fill:#66A,stroke:#225,stroke-width:2px,color:#fff
classDef service opacity:0.9,fill:#3B8062,stroke:#3B6062,stroke-width:2px,color:#fff
classDef topic opacity:0.9,fill:#852,stroke:#CCC,stroke-width:2px,color:#fff
classDef main opacity:0.9,fill:#059,stroke:#09F,stroke-width:4px,color:#fff
classDef bugged opacity:0.9,fill:#933,stroke:#800,stroke-width:2px,color:#fff
style keys opacity:0.15,fill:#FFF
style nodes opacity:0.15,fill:#FFF
style connection opacity:0.15,fill:#FFF
```
```
flowchart LR
/turtlesim:::main
/teleop_turtle:::node
/turtle1cmd_vel([/turtle1cmd_vel<br>geometry_msgs/msg/Twist]):::topic
/turtle1color_sensor([/turtle1color_sensor<br>turtlesim/msg/Color]):::bugged
/turtle1pose([/turtle1pose<br>turtlesim/msg/Pose]):::bugged
/clear[//clear<br>std_srvs/srv/Empty\]:::bugged
/kill[//kill<br>turtlesim/srv/Kill\]:::bugged
/reset[//reset<br>std_srvs/srv/Empty\]:::bugged
/spawn[//spawn<br>turtlesim/srv/Spawn\]:::bugged
/turtle1set_pen[//turtle1set_pen<br>turtlesim/srv/SetPen\]:::bugged
/turtle1teleport_absolute[//turtle1teleport_absolute<br>turtlesim/srv/TeleportAbsolute\]:::bugged
/turtle1teleport_relative[//turtle1teleport_relative<br>turtlesim/srv/TeleportRelative\]:::bugged
/turtle1/rotate_absolute{{/turtle1/rotate_absolute<br>turtlesim/action/RotateAbsolute}}:::action
/clear o-.-o /turtlesim
/kill o-.-o /turtlesim
/reset o-.-o /turtlesim
/spawn o-.-o /turtlesim
/turtle1set_pen o-.-o /turtlesim
/turtle1teleport_absolute o-.-o /turtlesim
/turtle1teleport_relative o-.-o /turtlesim
/teleop_turtle <==> /turtle1/rotate_absolute
/turtle1/rotate_absolute o==o /turtlesim
/turtle1cmd_vel --> /turtlesim
/turtlesim --> /turtle1color_sensor
/turtlesim --> /turtle1pose
/teleop_turtle --> /turtle1cmd_vel
subgraph keys[<b>Keys<b/>]
subgraph nodes[<b><b/>]
topicb((No connected)):::bugged
main_node:::main_node
end
subgraph connection[<b><b/>]
node1:::node
node2:::node
node1 o-. to server .-o service[/Service<br>service/Type\]:::service
service <-. to client .-> node2
node1 -- publish --> topic([Topic<br>topic/Type]):::topic
topic -- subscribe --> node2
node1 o== to server ==o action{{/Action<br>action/Type/}}:::action
action <== to client ==> node2
end
end
classDef node opacity:0.9,fill:#2A0,stroke:#391,stroke-width:4px,color:#fff
classDef action opacity:0.9,fill:#66A,stroke:#225,stroke-width:2px,color:#fff
classDef service opacity:0.9,fill:#3B8062,stroke:#3B6062,stroke-width:2px,color:#fff
classDef topic opacity:0.9,fill:#852,stroke:#CCC,stroke-width:2px,color:#fff
classDef main opacity:0.9,fill:#059,stroke:#09F,stroke-width:4px,color:#fff
classDef bugged opacity:0.9,fill:#933,stroke:#800,stroke-width:2px,color:#fff
style keys opacity:0.15,fill:#FFF
style nodes opacity:0.15,fill:#FFF
style connection opacity:0.15,fill:#FFF
```
## Custom style

For custom Mermaid styles, use the `--styleConfig` flag to load a style `.yaml` file. It should look like this (example uses default values; any omitted field will use its default):

```yaml
shapes:
  main: ["[", "]"]
  node: ["[", "]"]
  topic: ["([", "])"]
  service: ["[/", "\]"]
  action: ["{{", "}}"]
colors:
  main: "opacity:0.9,fill:#059,stroke:#09F,stroke-width:4px,color:#fff"
  node: "opacity:0.9,fill:#2A0,stroke:#391,stroke-width:4px,color:#fff"
  topic: "opacity:0.9,fill:#852,stroke:#CCC,stroke-width:2px,color:#fff"
  service: "opacity:0.9,fill:#3B8062,stroke:#3B6062,stroke-width:2px,color:#fff"
  action: "opacity:0.9,fill:#66A,stroke:#225,stroke-width:2px,color:#fff"
  no_conected: "opacity:0.9,fill:#933,stroke:#800,stroke-width:2px,color:#fff"
links_display:
  topics_publisher: "-->"
  topics_subscriber: "-->"
  services_server: "o-.-o"
  services_client: "<-.->"
  action_server: "o==o"
  action_client: "<==>"
links_style:
  topics_publisher:
  topics_subscriber:
  services_server:
  services_client:
  action_server: fill:none,stroke:green;
  action_client: fill:none,stroke:green;
display_keys: True
ignore:
  nodes:
    - "/Graph_generator"
  topics:
  services:
  actions:
```

Use the `ignore` field to exclude specific nodes, topics, services, or actions from your graph. [Regular expressions](https://docs.python.org/3/library/re.html) are supported.

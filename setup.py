import setuptools

with open("README.rst") as readme:
    long_description = readme.read()


setuptools.setup(
    name="ros2_graphs_mermaid",
    version="0.0.5",
    url="https://github.com/kiwicampus/ros2_graphs_mermaid",
    description="Generate mermaid description of ROS2 graphs to add on your markdown files.",
    long_description=long_description,
    packages=["ros2_graphs_mermaid"],
)

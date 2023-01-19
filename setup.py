import setuptools
from pip._internal.req import parse_requirements
from ros2_graphs_mermaid import __version__

with open("README.rst") as readme:
    long_description = readme.read()

install_reqs = list(parse_requirements("requirements.txt",session=False))

try:
    requirements = [str(ir.req) for ir in install_reqs]
except:
    requirements = [str(ir.requirement) for ir in install_reqs]

setuptools.setup(
    name="ros2_graphs_mermaid",
    version=__version__,
    url="https://github.com/kiwicampus/ros2_graphs_mermaid",
    description="Generate mermaid description of ROS2 graphs to add on your markdown files.",
    long_description=long_description,
    packages=["ros2_graphs_mermaid"],
    install_requires=requirements,
)

import setuptools
from pip._internal.req import parse_requirements

from ros2_graph import __version__

with open("README.md") as readme:
    long_description = readme.read()

install_reqs = list(parse_requirements("requirements.txt", session=False))

try:
    requirements = [str(ir.req) for ir in install_reqs]
except:
    requirements = [str(ir.requirement) for ir in install_reqs]

setuptools.setup(
    name="ros2_graph",
    version=__version__,
    url="https://github.com/kiwicampus/ros2_graph",
    description="Generate mermaid description of ROS2 graphs to add on your markdown files.",
    long_description=long_description,
    long_description_content_type='text/markdown',
    packages=["ros2_graph"],
    install_requires=requirements,
    entry_points={"console_scripts": ["ros2_graph = ros2_graph:__main__.main"]},
)

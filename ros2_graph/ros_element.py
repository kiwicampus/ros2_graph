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

from dataclasses import dataclass
from enum import Enum
from typing import List, Dict


class ElementType(Enum):
    """!
    Enumeration class for ROS element types
    """

    MAIN = 0
    NODE = 1
    TOPIC = 2
    SERVICE = 3
    ACTION = 4


def element_style(element_type: ElementType) -> str:
    """!
    Just map element types and and it's string
    """
    if element_type == ElementType.MAIN:
        return "main"
    if element_type == ElementType.NODE:
        return "node"
    if element_type == ElementType.TOPIC:
        return "topic"
    if element_type == ElementType.SERVICE:
        return "service"
    if element_type == ElementType.ACTION:
        return "action"
    return "default"


class LinkType(Enum):
    """!
    Enumeration class for ROS relationships types
    """

    TOPIC_PUBLISHER = 0
    TOPIC_SUBSCRIBER = 1
    SERVICE_SERVER = 2
    SERVICE_CLIENT = 3
    ACTION_SERVER = 4
    ACTION_CLIENT = 5

    @classmethod
    def inverse_link(cls, link_type: int) -> int:
        """!
        Map "from" links to "to" links and vice versa
        """
        if link_type == LinkType.TOPIC_SUBSCRIBER:
            return LinkType.TOPIC_PUBLISHER
        if link_type == LinkType.TOPIC_PUBLISHER:
            return LinkType.TOPIC_SUBSCRIBER
        if link_type == LinkType.SERVICE_SERVER:
            return LinkType.SERVICE_CLIENT
        if link_type == LinkType.SERVICE_CLIENT:
            return LinkType.SERVICE_SERVER
        if link_type == LinkType.ACTION_SERVER:
            return LinkType.ACTION_CLIENT
        if link_type == LinkType.ACTION_CLIENT:
            return LinkType.ACTION_SERVER
        raise ValueError("Non valid link type code")


@dataclass
class RosElement:
    """! 
        RosElement Class store basic information of ROS elements: 
        Nodes, topics, services and actions
    """

    name: str
    namespace: str
    type: ElementType

    def full_name(self) -> str:
        """!  Put togheter name and name space on a sigle string
        @return str namespace/name
        """
        if self.namespace is None:
            return self.name
        return self.namespace + self.name


def list_ros_element_2_list_str(elements: List[RosElement]) -> List[str]:
    """!
    convert a RosElement list to its string list representation
    """
    return list(map(str, elements))


@dataclass
class NoNodeElement(RosElement):
    """!
        NoNodeElement Class store aditional information of ROS elements 
        that are not nodes.
        Atributes:
            ros_type: ros topic, service or action type
            from_connected: bool, has at least one publisher or server
            to_connected: bool, has at least one subscriber or client
            brackets: for mermaid diplay, default ("([", "])")
    """

    ros_type: str
    from_connected: bool = False
    to_connected: bool = False
    brackets: List[str] = ("([", "])")

    def __str__(self) -> str:
        name = self.full_name()
        style = (
            element_style(self.type)
            if self.from_connected and self.to_connected
            else "bugged"
        )

        return f"{name}{self.brackets[0]} {name}<br>{self.ros_type} {self.brackets[1]}:::{style}"

    def __eq__(self, another) -> bool:
        return self.name == another.name and self.namespace == another.namespace

    def __hash__(self) -> int:
        return hash(self.name) ^ hash(self.namespace)


@dataclass
class Link:
    """!
        contains the relashionship information from a ROS node
        to some other ROS element
        Atributes:
            linked_element: NoNodeElement object
            link_str: for mermaid display
    """

    linked_element: NoNodeElement
    link_str: str

    def __str__(self) -> str:
        return self.linked_element.full_name()

    def __eq__(self, another) -> bool:
        return (
            self.linked_element == another.linked_element
            and self.link_str == another.link_str
        )

    def __hash__(self) -> int:
        return hash(self.linked_element) ^ hash(self.link_str)


@dataclass
class NodeElement(RosElement):
    """!
        NoNodeElement Class store aditional information of ROS nodes.
        Atributes:
            brackets: for mermaid diplay
    """

    brackets: List[str]

    def __post_init__(self):
        self.links: List[Dict[int, Link]] = [dict() for i in range(6)]

    def add_link(
        self, linked_element: NoNodeElement, link_str: str, link_type: LinkType
    ):
        """!
         Add a relashion ship  with a no node element,
         but only if it doesn't already exist
         @param linked_element: a topic, sevice or action
         @param link_str: for mermaid display
         @param link_type: which kind of relashionship is
        """
        link = Link(linked_element=linked_element, link_str=link_str)
        link_hash = hash(link)
        index = link_type.value
        if link_hash not in self.links[index]:
            self.links[index][link_hash] = link
            if index % 2:
                linked_element.from_connected = True
            else:
                linked_element.to_connected = True

    def get_links_str(self, which: LinkType) -> List[str]:
        """!
        Get al links string representation from a desired type
        @param which: specify the link type
        """
        name = self.full_name()
        index = which.value
        linksStr = [
            name + " " + link.link_str + " " + str(link)
            if index % 2
            else str(link) + " " + link.link_str + " " + name
            for link in self.links[index].values()
        ]
        return linksStr

    def __str__(self):
        name = self.full_name()
        return f"{name}{self.brackets[0]} {name} {self.brackets[1]}:::{element_style(self.type)}"

    def __eq__(self, another) -> bool:
        return self.name == another.name and self.namespace == another.namespace

    def __hash__(self) -> int:
        return hash(self.name) ^ hash(self.namespace)

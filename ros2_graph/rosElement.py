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
from typing import List, Set


class ELEMENT_TYPE(Enum):
    MAIN = 0
    NODE = 1
    TOPIC = 2
    SERVICE = 3
    ACTION = 4


def element_style(element_type: int) -> str:
    if element_type == ELEMENT_TYPE.MAIN:
        return "main"
    if element_type == ELEMENT_TYPE.NODE:
        return "node"
    if element_type == ELEMENT_TYPE.TOPIC:
        return "topic"
    if element_type == ELEMENT_TYPE.SERVICE:
        return "service"
    if element_type == ELEMENT_TYPE.ACTION:
        return "action"
    return "default"


class LINK_TYPE(Enum):
    TOPIC_SUBSCRIBER = 0
    TOPIC_PUBLISHER = 1
    SERVICE_SERVER = 2
    SERVICE_CLIENT = 3
    ACTION_SERVER = 4
    ACTION_CLIENT = 5


@dataclass
class RosElement:
    # RosElement Class store basic information of ROS elements: Nodes, topics, services and actions
    name: str
    namespace: str
    type: int

    def full_name(self) -> str:
        """!  Put togheter name and name space on a sigle string
        @return str namespace/name
        """
        if self.namespace is None:
            return self.name
        if len(self.namespace) > 1:
            self.namespace += "/"
        return self.namespace + self.name

    def addTypes(self, types) -> None:
        self.types = types

    def __eq__(self, other) -> bool:
        return (
            self.name == other.name
            and self.namespace == other.namespace
            and self.type == other.type
        )

    def __hash__(self) -> int:
        return hash(self.name) ^ hash(self.namespace) ^ hash(self.type)


@dataclass
class NoNodeElement(RosElement):
    ros_type: str
    no_connected: bool = True

    def __str__(self) -> str:
        name = self.full_name()
        style = "bugged" if self.no_connected else element_style(self.type)
        if self.type == ELEMENT_TYPE.TOPIC:
            brackets = ("([", "])")
        if self.type == ELEMENT_TYPE.SERVICE:
            brackets = ("[/", "\]")
        if self.type == ELEMENT_TYPE.ACTION:
            brackets = ("{{", "}}")

        return (
            name
            + brackets[0]
            + name
            + "<br>"
            + self.ros_type
            + brackets[1]
            + ":::"
            + style
        )


@dataclass
class Link:
    linkedElement: NoNodeElement
    linkStr: str

    def __str__(self) -> str:
        return self.linkStr + " " + self.linkedElement.full_name()

    def __eq__(self, other) -> bool:
        return self.linkedElement and self.linkStr

    def __hash__(self) -> int:
        return hash(self.linkedElement) ^ hash(self.linkStr)


@dataclass
class NodeElement(RosElement):
    links: List[Set[Link]] = [set()] * 6

    def addLink(self, linkedElement: NoNodeElement, linkStr: str, link_type: int):
        link = Link(linkedElement=linkedElement, linkStr=linkStr)
        self.links[link_type].add(link)

    def getLinksStr(self, which: int) -> str:
        name = self.full_name()
        return [name + " " + str(link) for link in self.links[which]]

    def __str__(self):
        name = self.full_name()
        return f"{name}:::{element_style(self.type)}"

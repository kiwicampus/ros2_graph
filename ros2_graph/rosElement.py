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
    TOPIC_PUBLISHER = 0
    TOPIC_SUBSCRIBER = 1
    SERVICE_SERVER = 2
    SERVICE_CLIENT = 3
    ACTION_SERVER = 4
    ACTION_CLIENT = 5

    def inverse_link(link_type: int) -> int:
        if link_type == LINK_TYPE.TOPIC_SUBSCRIBER:
            return LINK_TYPE.TOPIC_PUBLISHER
        if link_type == LINK_TYPE.TOPIC_PUBLISHER:
            return LINK_TYPE.TOPIC_SUBSCRIBER
        if link_type == LINK_TYPE.SERVICE_SERVER:
            return LINK_TYPE.SERVICE_CLIENT
        if link_type == LINK_TYPE.SERVICE_CLIENT:
            return LINK_TYPE.SERVICE_SERVER
        if link_type == LINK_TYPE.ACTION_SERVER:
            return LINK_TYPE.ACTION_CLIENT
        if link_type == LINK_TYPE.ACTION_CLIENT:
            return LINK_TYPE.ACTION_SERVER
        raise ValueError("Non valid link type code")


@dataclass
class RosElement:
    # RosElement Class store basic information of ROS elements: Nodes, topics, services and actions
    name: str
    namespace: str
    type: ELEMENT_TYPE

    def full_name(self) -> str:
        """!  Put togheter name and name space on a sigle string
        @return str namespace/name
        """
        if self.namespace is None:
            return self.name
        return self.namespace + self.name

    def addTypes(self, types) -> None:
        self.types = types


def listRosElement2ListStr(elements: List[RosElement]) -> List[str]:
    return [str(element) for element in elements]


@dataclass
class NoNodeElement(RosElement):
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

        return (
            name
            + self.brackets[0]
            + name
            + "<br>"
            + self.ros_type
            + self.brackets[1]
            + ":::"
            + style
        )

    def __eq__(self, another) -> bool:
        return self.name == another.name and self.namespace == another.namespace

    def __hash__(self) -> int:
        return hash(self.name) ^ hash(self.namespace)


@dataclass
class Link:
    linkedElement: NoNodeElement
    linkStr: str

    def __str__(self) -> str:
        return self.linkedElement.full_name()

    def __eq__(self, another) -> bool:
        return (
            self.linkedElement == another.linkedElement
            and self.linkStr == another.linkStr
        )

    def __hash__(self) -> int:
        return hash(self.linkedElement) ^ hash(self.linkStr)


@dataclass
class NodeElement(RosElement):
    brackets: List[str]

    def __post_init__(self):
        self.links: List[Dict[int, Link]] = [dict() for i in range(6)]

    def addLink(self, linkedElement: NoNodeElement, linkStr: str, link_type: LINK_TYPE):
        link = Link(linkedElement=linkedElement, linkStr=linkStr)
        link_hash = hash(link)
        index = link_type.value
        if link_hash not in self.links[index]:
            self.links[index][link_hash] = link
            if index % 2:
                linkedElement.from_connected = True
            else:
                linkedElement.to_connected = True

    def getLinksStr(self, which: LINK_TYPE) -> List[str]:
        name = self.full_name()
        index = which.value
        linksStr = [
            name + " " + link.linkStr + " " + str(link)
            if index % 2
            else str(link) + " " + link.linkStr + " " + name
            for link in self.links[index].values()
        ]
        return linksStr

    def __str__(self):
        name = self.full_name()
        return f"{name}:::{element_style(self.type)}"

    def __eq__(self, another) -> bool:
        return self.name == another.name and self.namespace == another.namespace

    def __hash__(self) -> int:
        return hash(self.name) ^ hash(self.namespace)

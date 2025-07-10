from __future__ import annotations

from urdf_parser_py import urdf
from typing import List

robot = urdf.Robot.from_xml_file("robot-urdfs/abb_irb6700_150_320/abb_irb6700_150_320.urdf")
linear_axis = urdf.Robot.from_xml_file("robot-urdfs/linear_axis/linear_axis.urdf")


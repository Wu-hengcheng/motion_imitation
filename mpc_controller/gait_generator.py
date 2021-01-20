"""Gait pattern planning module.
   步态模式计划模块。
"""

from __future__ import absolute_import
from __future__ import division
#from __future__ import google_type_annotations
from __future__ import print_function

import abc
import enum


class LegState(enum.Enum):
  """The state of a leg during locomotion."""
  SWING = 0
  STANCE = 1
  # A swing leg that collides with the ground.
  EARLY_CONTACT = 2
  # A stance leg that loses contact.
  LOSE_CONTACT = 3


class GaitGenerator(object):  # pytype: disable=ignored-metaclass
  """Generates the leg swing/stance pattern for the robot.
    为机器人生成腿部摆动/站立模式。
  """

  __metaclass__ = abc.ABCMeta

  @abc.abstractmethod
  def reset(self, current_time):
    pass

  @abc.abstractmethod
  def update(self, current_time):
    pass

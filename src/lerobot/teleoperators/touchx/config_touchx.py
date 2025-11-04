#!/usr/bin/env python

"""Configuration dataclass for the Touch X teleoperator adapter."""

from __future__ import annotations

from dataclasses import dataclass

from ..config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("touchx")
@dataclass(kw_only=True)
class TouchXTeleopConfig(TeleoperatorConfig):
    """Configuration options for the Touch X teleoperation device."""

    device_name: str = "Default Device"
    scheduler_type: str = "async"
    # TODO(user): tune the translation scaling factor to match desired hand-to-robot motion.
    translation_scale: float = 100
    # TODO(user): tune the rotation scaling factor once rotational control is validated.
    rotation_scale: float = 0
    # Low-pass smoothing factor for translation deltas (0 disables, 1 = no smoothing).
    translation_smoothing_alpha: float = 0.2
    # Low-pass smoothing factor for rotation deltas (0 disables, 1 = no smoothing).
    rotation_smoothing_alpha: float = 0.2
    # Default absolute gripper state (0 = open, 1 = closed).
    default_gripper_state: float = 1000.0

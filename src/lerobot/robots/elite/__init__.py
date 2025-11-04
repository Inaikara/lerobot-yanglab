"""Elite robot adapter exports."""

from .config_elite import EliteConfig
from .elite_robot import EliteRobot

class Elite(EliteRobot):
    pass

__all__ = ["EliteConfig", "EliteRobot"]


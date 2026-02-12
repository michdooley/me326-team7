"""Reusable subtask modules for TidyBot2 task orchestration."""

from enum import Enum


class SubtaskStatus(Enum):
    RUNNING = 0
    SUCCEEDED = 1
    FAILED = 2


from .scan_for_object import ScanForObject
from .navigate_to import NavigateTo
from .pick_up_object import PickUpObject

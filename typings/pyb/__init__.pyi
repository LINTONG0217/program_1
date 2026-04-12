from machine import Pin as Pin
from machine import PWM as PWM
from machine import UART as UART


def delay(ms: int) -> None: ...

def millis() -> int: ...

def elapsed_millis(start: int) -> int: ...

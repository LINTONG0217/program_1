class _FPIOA:
    UART1_TX: int
    UART1_RX: int


class _FM:
    fpioa: _FPIOA
    def register(self, pin, function, force=False): ...


fm: _FM

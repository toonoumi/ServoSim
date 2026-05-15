import math

# Standard 3-phase hall truth table: 6 sectors of 60° each per electrical cycle
_HALL_TABLE = [
    (1, 0, 0),
    (1, 1, 0),
    (0, 1, 0),
    (0, 1, 1),
    (0, 0, 1),
    (1, 0, 1),
]


class HallSensor:
    def __init__(self, poles: int = 7):
        self.poles = poles
        self.hall_a: int = 1
        self.hall_b: int = 0
        self.hall_c: int = 0

    def update(self, position_deg: float) -> None:
        elec_angle = (position_deg / 360.0) * self.poles * 360.0
        sector = int(math.floor(elec_angle / 60.0)) % 6
        self.hall_a, self.hall_b, self.hall_c = _HALL_TABLE[sector]

    @property
    def state(self) -> tuple:
        return (self.hall_a, self.hall_b, self.hall_c)

from dataclasses import dataclass, field

@dataclass
class SolverInterface:
    type: str = 'ipopt'
    receding: bool = False
    opts: dict = field(default_factory=dict)


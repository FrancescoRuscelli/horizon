from typing import TypedDict, Iterable, Union, Optional
from numpy.typing import ArrayLike
import casadi as cs
class BoundsDict(TypedDict):
    """
    nodes, lb, ub
    """
    nodes: Optional[Iterable]
    lb: Union[ArrayLike, Iterable, cs.DM]
    ub: Union[ArrayLike, Iterable, cs.DM]
from typing import TypedDict, Iterable, Union, Optional
import casadi as cs
# from numpy.typing import ArrayLike
class BoundsDict(TypedDict):
    """
    nodes, lb, ub
    """
    nodes: Optional[Iterable]
    lb: Union[Iterable, cs.DM] # ArrayLike
    ub: Union[Iterable, cs.DM]
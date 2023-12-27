from typing import List
from typing import Union


def rcb4_checksum(byte_list: List[int]) -> int:
    """Calculates the checksum for a list of byte values.

    The checksum is calculated as the sum of all byte values,
    each masked with 0xff, and then the result is masked with 0xff.

    Parameters
    ----------
    byte_list : list of int
        The list of byte values for which the checksum is to be calculated.

    Returns
    -------
    int
        The calculated checksum.
    """
    return sum(b & 0xff for b in byte_list) & 0xff


def rcb4_servo_ids_to_5bytes(seq: List[int]) -> List[int]:
    ids = [0, 0, 0, 0, 0]
    for c in seq:
        ids[c // 8] |= (1 << (c % 8))
    return ids


def rcb4_velocity(v):
    return min(255, int(round(v)))


def rcb4_servo_positions(
        ids: Union[int, List[int]], fvector: List[float]) -> List[int]:
    """Creates a buffer with servo positions from given ids and float vector.

    Parameters
    ----------
    ids : list or similar
        A list of ids corresponding to servo positions.
    fvector : list
        A list of floating-point values representing servo positions.

    Returns
    -------
    list
        A list of bytes representing the low and high bytes of servo positions.
    """
    if not isinstance(ids, list):
        ids = list(ids)

    fv = [int(round(v)) for v in fvector]
    buf = []
    for d in fv:
        buf.append(d & 0xff)
        buf.append((d >> 8) & 0xff)
    return buf


def four_bit_to_num(lst: List[int], values: List[int]):
    result = 0
    for index in lst:
        result = (result << 4) | (values[index - 1] & 0x0f)
    return result


def rcb4_servo_svector(ids: List[int], svector: List[float]) -> List[int]:
    return [int(round(v)) & 0xff
            for _, v in zip(ids, svector)]

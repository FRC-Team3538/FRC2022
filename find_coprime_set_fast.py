import numpy as np
import itertools

from typing import List, Optional


# depth: returns sets of depth + 2 numbers that are coprime with each other
def recurse(stack: List[int], depth=0) -> Optional[List[int]]:
    for test in reversed(np.arange(stack[-1])):
            
        good = True
        for stack_value in stack:
            if np.gcd(stack_value, test) != 1:
                good = False
                break
        if good:
            stack.append(test)
            if depth == 0:
                return stack
            res = recurse(stack, depth - 1)
            if res is not None:
                return res
            stack.pop()
    return None


stack = []

for i in reversed(np.arange(256)):
    print('trying', i)
    stack.append(i)
    res = recurse(stack, 13)
    if res is not None:
        print(res)
        break
    stack.pop()

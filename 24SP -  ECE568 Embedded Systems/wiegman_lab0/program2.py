#!/usr/bin/python3

# because `from .program3 import fibonacci` didn't work, for some reason:
def fibonacci(fib_length):
    seq = [1, 1]
    while len(seq) < fib_length:
        seq.append(seq[-2] + seq[-1])
    return seq
# ----

a = fibonacci(11)
print(f"{a=}")
ceiling = int(input("Enter number: "))
print(f"The new list is {[x for x in a if x < ceiling]}")

#!/usr/bin/python3

out_length = int(input("How many Fibonacci numbers would you like to generate? "))

def fibonacci(fib_length):
    seq = [1, 1]
    while len(seq) < fib_length:
        seq.append(seq[-2] + seq[-1])
    return seq

print(f"The Fibonacci Sequence is: {fibonacci(out_length)}")

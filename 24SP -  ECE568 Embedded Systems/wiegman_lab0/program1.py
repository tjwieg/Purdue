#!/usr/bin/python3

CURRENT_YEAR = 2024
name = input("What is your name? ")
age = input("How old are you? ")

centennial = (100-int(age)) + CURRENT_YEAR

print(f"{name} will turn 100 years old in either {centennial-1} or {centennial}.")

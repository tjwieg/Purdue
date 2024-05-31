#!/usr/bin/python3

BIRTHDAYS = {
    "Albert Einstein":   "1879-03-14",
    "Benjamin Franklin": "1706-01-17",
    "Ada Lovelace":      "1815-12-10"
}

print("Welcome to the birthday dictionary. We know the birthdays of:")
for person in BIRTHDAYS: print("\t - " + person)

name = input("Whose birthday do you want to look up? ")
if name in BIRTHDAYS:
    print(f"{name}'s birthday is {BIRTHDAYS[name]}")
else:
    print("Sorry, we don't know that person.")

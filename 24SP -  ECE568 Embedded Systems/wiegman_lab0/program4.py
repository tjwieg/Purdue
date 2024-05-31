#!/usr/bin/python3

from random import randint
winner = randint(0,10)

for i in range(3):
    guess = input("Enter your guess: ")
    if guess == str(winner):
        win = True
        break
    else: win = False

if win: print("You win!")
else: print(f"You lose! (The correct answer was {winner})")

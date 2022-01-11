#play again
def play_again():
    print("\nDo you want to play again? (y or n)")
    answer = input(">").lower()

    if "y" in answer:
        start()
    else:
        exit()        

#game over
def game_over(reason):
    print("\n" + reason)
    print("Game over!")
    play_again()

#diamond room
def diamond_room():
    print("\nYou are now in a room filed with diamonds!")
    print("And there is a door too!")
    print("What would you do? (1 or 2)")
    print("1). Take some diamonds and go through the door.")
    print("2). Just go through the door.")

    answer = input(">")

    if answer == "1":
        game_over("They were cursed diamonds! The moment you touched it, the building collapsed, and you die!")
    elif answer == "2":
        print("\nNice, you are an honest man! Congratulations. You win the game.")
        play_again()
    else:
        game_over("Go and learn how to type a number")

#monster room
def monster_room():
    print("\nNow you entered the room of a monster!")
    print("The monster is sleeping.\nBehind the monster, there is another door. What would you do? (1 or 2)")
    print("1). Go through the door silently.")
    print("2). Kill the monster and show your courage!")

    answer = input(">")

    if answer == "1":
        diamond_room()
    elif answer == "2":
        game_over("The monster was hungry, he/it ate you.")
    else:
        game_over("Go and learn how to type a number")

#bear room
def bear_room():
    #give some prompts
    print("\nThere is a bear here")
    print("Behind the bear is another door")
    print("The bear is eating tasty honey!")
    print("What would you do? (1 or 2)")
    print("1). Take the honey.")
    print("2). Taunt the bear.")

    answer = input(">")

    if answer == "1":
        game_over("The bear killed you")
    elif answer == "2":
        print("\nYour good time, the bear moved from the door. You can go through it now!")
        diamond_room()
    else:
        game_over("Dont you know how to type a number")

def start():
    #give some prompts
    print("\nYou are standing in a dark room.")
    print("There is a door to your left and right, which one do you choose? (l or r)")

    #convert user input to lower case
    answer = input(">").lower()

    if "l" in answer:
        #if player typed "left" or "l" proceed to left door to bear_room()
        bear_room()
    elif "r" in answer:
        #else player typed "right" or "r" proceed to right door to monster_room()
        monster_room()
    else:
        #else call game_over() function with the "reason" argument
        game_over("Dont you know how to type something?")

#start the game
start()
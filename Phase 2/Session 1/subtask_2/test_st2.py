from line_saver import save_sentence

#######################################
# if not in folder subtask_2
# then write in terminal: cd subtask_2
# before running the program
#######################################

print("enter lines to save ... press ctrl+c to exit")

try:
    while(True):
        sentence = input("enter sentence: ")
        if not save_sentence(sentence):
            print("line not saved, error occured")
except KeyboardInterrupt:
    print("ctrl+c was pressed. Exiting...")
except:
    print("ERROR")

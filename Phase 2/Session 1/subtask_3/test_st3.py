from save_data import data_saver


#######################################
# if not in folder subtask_2
# then write in terminal: cd subtask_2
# before running the program
#######################################


print("enter dictionary values ... press ctrl+c to save and exit")

data = {}
file_type = 'json'

try:
    while True:
        json_temp = input("Do you want jason file? (enter y or n)\n")
        if json_temp == 'y':
            file_type = 'json'
            break
        elif json_temp == 'n':
            file_type = 'txt'
            break
        else:
            json_temp = input("Invalid input, enter y for json and n for txt files")
            
    while(True):
        key = input("enter key: ")
        value = input("enter value: ")
        data[key] = value
        # if not data_saver(sentence):
        #     print("line not saved, error occured")

except KeyboardInterrupt:
    print("ctrl+c was pressed. Saving...")
    if not data_saver(data, file_type):
        print("ERROR while saving in file");
    else:
        print("Saved Successfully :)")

except:
    print("ERROR")
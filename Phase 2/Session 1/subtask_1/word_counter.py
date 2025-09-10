import re

def count_words(sentence: str, case_sensitive=False) -> dict[str, int]:
    word_count_dict = {}
    str_list = re.split(r'[ ,;./\?"]+', sentence) # turn string into list
    
    for word in str_list:
        if case_sensitive == False:
            word = word.lower()
        if(word_count_dict.get(word) == None):
            word_count_dict[word] = 1
        else:
            word_count_dict[word] += 1

    return word_count_dict
# Fn done
from pathlib import Path

def save_sentence(sentence: str, file_name = 'output.txt') -> bool:
    file_path = Path(file_name)
    try:
        with file_path.open('a') as file:
            file.write(sentence + '\n')
    except:
        return False
    return True

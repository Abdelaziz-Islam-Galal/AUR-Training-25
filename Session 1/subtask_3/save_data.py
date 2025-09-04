from pathlib import Path
import json

def data_saver(data: dict, data_format = 'json', file_name = 'output.') -> bool:
    if not data_format == 'txt' and not data_format == 'json':
        print("Invalid file type")
        return False
    
    file_name += data_format
    file_path = Path(file_name)

    print(f"Attempting to create file at: {file_path.absolute()}") # testing

    if data_format == 'json':
        with file_path.open('w') as file:
            json.dump(data, file)
    elif data_format == 'txt':
        try:
            with file_path.open('a') as file:
                for key, value in data.items():
                    file.write(str(key) + ' = ' + str(value) + '\n')
        except:
            print("ERROR in creating or opening file")
            return False
    return True
from threading import Thread
from time import sleep # sleep will simulate having a function that takes a lot of time to finish
from functools import partial

def download(filename):
    sleep(5)
    print(f"{filename} has finished downloading")

def download_threaded(filename):
    thread = Thread(target=partial(download, filename), daemon = True) # if there is no parameters in download just write target=download without using partial
    # daemon = True is used to make the main thread to terminate without waiting for the other threads
    # so the program terminates even before the print in the first download runs
    print(f"{filename} has started downloading")
    thread.start()

download_threaded("file1")
download_threaded("file2")
download_threaded("file3")


# The next 5 lines are written (once) so when making the project into an executable file, the absolute imports work
# absolute imports is when somone imports his own code into ather part of the same code
# like whaat we did in the line: "from RobotGui.gui.window import Window"
import sys
if __package__ is None and not getattr(sys, 'frozen', False):
    import os.path
    path = os.path.realpath(os.path.abspath(__file__))
    sys.path.insert(0, os.path.dirname(os.path.dirname(path)))

from PySide6.QtWidgets import QApplication
from RobotGui.gui.window import Window

app = QApplication() # creating application

window = Window() # the window that will appear in the application

app.exec() # executing the applocation
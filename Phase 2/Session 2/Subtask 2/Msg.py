from time import time
from rich.console import Console

console = Console()


def send_Msg(msg):
    if isinstance(msg, BaseMsg):
        console.print(msg, style = msg.style)
    else:
        print(msg)


class BaseMsg:
    def __init__(self, data: str):
        self._data = data
    
    @property
    def style(self):
        return '' # BaseMsg-specific
        
    @property
    def data(self):
        return self._data
    
    def __str__(self):
        return self._data # BaseMsg-specific
    
    def __len__(self):
        return len(self.__str__())
    
    def __eq__(self, other):
        return isinstance(other, BaseMsg) and self.__str__() == other.__str__()
    
    def __add__(self, other):
        if isinstance(other, BaseMsg):
            ans = self._data + other._data
            # I am using data because it will return BasMsg type that will have its own __str__
            return BaseMsg(ans)
        else:
            try:
                ans = self._data + str(other)
                return BaseMsg(ans)
            except Exception:
                raise TypeError("incompatible type for addition")

class LogMsg(BaseMsg):
    def __init__(self, data:str):
        super().__init__(data)
        self._timestamp: int = int(time()) # erase dots and assign value to use it in __str__()
    
    @property
    def style(self): # type: ignore
        return "on yellow"

    def __str__(self):
        return f"[{self._timestamp}] " + self._data


class WarnMsg(LogMsg):
    @property
    def style(self): # type: ignore
        return "white on red"

    def __str__(self):
        return f"[!WARN][{self._timestamp}] " + self._data


if __name__ == '__main__':

    m1 = BaseMsg('Normal message')
    m2 = LogMsg('Log')
    m3 = WarnMsg('Warning')
    send_Msg(m1)
    send_Msg(m2)
    send_Msg(m3)

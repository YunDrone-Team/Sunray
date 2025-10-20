import sys
import time

class LogColor:
    def_ = 0
    red = 1
    green = 2
    yellow = 3
    blue = 4
    magenta = 5
    cyan = 6
    white = 7
    black = 8
    # 可继续扩展

class Logger:
    colors = [
        "\033[0m", "\033[31m", "\033[32m", "\033[33m", "\033[34m", "\033[35m", "\033[36m", "\033[37m", "\033[30m"
    ]
    levels = ["DEBUG", "INFO ", "WARN ", "ERROR", "QUIET"]
    delimiter = " "
    printLevel = True
    printTime = True
    printToFile = False
    fileName = "log.txt"
    is_init = False

    # 初始化分隔符
    @staticmethod
    def init_default():
        Logger.is_init = True

    #是否打印level
    @staticmethod
    def setPrintLevel(p):
        Logger.printLevel = p

    #是否打印时间
    @staticmethod
    def setPrintTime(p):
        Logger.printTime = p

    #是否打印到文件
    @staticmethod
    def setPrintToFile(p):
        Logger.printToFile = p

    #设置日志文件名
    @staticmethod
    def setFilename(f):
        Logger.fileName = f

    @staticmethod
    def print_color(color, node_name, *args):
        if not Logger.is_init:
            Logger.init_default()
        msg = Logger.colors[color]
        if Logger.printLevel:
            msg += "[INFO]"
        if Logger.printTime:
            msg += time.strftime("%Y-%m-%d %H:%M:%S") + ":"
        msg += Logger.delimiter + node_name + Logger.delimiter + " ".join(str(a) for a in args) + "\033[0m"
        print(msg)
        if Logger.printToFile:
            with open(Logger.fileName, "a") as f:
                f.write(msg + "\n")
import logging
import os
from logging.handlers import RotatingFileHandler
from typing import Union

LOG_DIR = "logs"
os.makedirs(LOG_DIR, exist_ok=True)

handlersExist = False

def createShortName(unfilterred: str) -> str:
    return unfilterred.split(".")[-1]

def setupLogger(name: str, log_file: str = "main.log", level=logging.INFO, consoleLevel = logging.WARNING) -> Union[logging.Logger, tuple[logging.Logger, logging.Logger]]:
    
    global handlersExist

    logger = logging.getLogger("globalLogger")
    logger.setLevel(level)
    logger.name = createShortName(name)
    if not handlersExist:
        #print(f"New logger instance -> {name}")
        #To much detail
        #formatter = logging.Formatter("%(asctime)s:%(processName)s:%(name)s:%(levelname)s -> %(message)s", datefmt="%Y-%m-%d %H:%M:%S")
        formatter = logging.Formatter("%(asctime)s:%(name)s:%(levelname)s -> %(message)s", datefmt="%Y-%m-%d %H:%M:%S")
        
        fileHandler = RotatingFileHandler(os.path.join(LOG_DIR, log_file), maxBytes=10**6, backupCount=5)
        fileHandler.setLevel(level)
        fileHandler.setFormatter(formatter)

        consoleHandler = logging.StreamHandler()
        consoleHandler.setLevel(consoleLevel)
        consoleHandler.setFormatter(formatter)

        logger.addHandler(fileHandler)
        logger.addHandler(consoleHandler)

        mainLogger = logging.getLogger("Main")
        mainLogger.setLevel(level)

        mainLogger.addHandler(fileHandler)
        mainLogger.addHandler(consoleHandler)

        handlersExist = True

        return logger, mainLogger

    return logger
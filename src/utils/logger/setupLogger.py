import logging.config
import logging
import json
import os
from typing import Union, Optional
from enum import Enum

class LoggerConfigs(Enum):
    """Allowed logger config types."""
    INIT = 'init.json'
    LISTENER = 'listener.json'
    WORKER = 'worker.json'

_LOG_DIR = 'logs'

def _createShortName(fullName: str) -> str:
    """(Private) Extracts the short name from a full module path."""
    return fullName.split(".")[-1]

def _loadJsonConfigFromFile(filename: str):
    """(Private) Loads logging config from a JSON file."""
    dirPath = os.path.dirname(os.path.realpath(__file__))
    dirPath = os.path.join(dirPath, "configs")

    filePath = os.path.join(dirPath, filename)

    with open(filePath) as file:
        return json.load(file)

def configLogger(loggerType :LoggerConfigs, loggerName: Optional[str] = None, queue = None) -> Union[logging.Logger, None]:
    """
    Configures a logger based on the given type.
    
    Args:
        loggerType: One of LoggerConfigs values (INIT, LISTENER, WORKER).
        loggerName: Name of the logger (required for INIT and WORKER).
        queue: Required if loggerType is WORKER.
    
    Raises:
        ValueError: If queue is missing for WORKER or invalid loggerType.
    """
    # Ensure queue is provided for WORKER
    if loggerType == LoggerConfigs.WORKER and queue is None:
        raise ValueError("Queue must be provided for WORKER logger")

    # Create log directory on INIT
    if loggerType == LoggerConfigs.INIT:
        os.makedirs(_LOG_DIR, exist_ok=True)

    #Check for worker logger type that queue is sent

    configJson = _loadJsonConfigFromFile(loggerType.value)
    if loggerType == LoggerConfigs.WORKER:
        configJson['handlers']['queue'] = {
        'class': 'logging.handlers.QueueHandler',
        'queue': queue
        }
    logging.config.dictConfig(configJson)

    if loggerType in (LoggerConfigs.INIT, LoggerConfigs.WORKER):
        if not loggerName:
            raise ValueError("loggerName must be provided for INIT and WORKER")
        return logging.getLogger(_createShortName(loggerName))
    else:
        return None

class LoggingHandler:

    def handle(self, record):
        if record.name == "root":
            logger = logging.getLogger()
        else:
            #print(f"Processing record for {record.name} at level {record.levelno}")
            logger = logging.getLogger(record.name)
            #print(f"Logger effective level: {logger.getEffectiveLevel()}")
            #print(f"Logger handlers: {logger.handlers}")

        if logger.isEnabledFor(record.levelno):
            logger.handle(record)
        else:
            pass
            #print("Record filtered due to level")
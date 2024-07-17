import logging
from functools import partial, partialmethod
from logging.handlers import RotatingFileHandler

# Add TRACE log level
logging.TRACE = 5
logging.addLevelName(logging.TRACE, "TRACE")
logging.Logger.trace = partialmethod(logging.Logger.log, logging.TRACE)
logging.trace = partial(logging.log, logging.TRACE)


def init_logging(level: str = "debug", filename: str = "./logs/velmwheel/default.log"):
    match level:
        case "info":
            level = logging.INFO
        case "debug":
            level = logging.DEBUG
        case "trace":
            level = logging.TRACE
        case _:
            raise ValueError(f"Unknown log level: {level}")

    # pylint: disable=redefined-builtin
    format = "%(asctime)s %(filename)s:%(lineno)d %(levelname)s %(message)s"

    handler = RotatingFileHandler(filename, maxBytes=20 * 1024 * 1024, backupCount=5)
    handler.setLevel(level)

    formatter = logging.Formatter(format)
    handler.setFormatter(formatter)

    root_logger = logging.getLogger()
    root_logger.setLevel(level)
    root_logger.addHandler(handler)

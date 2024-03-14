import logging
from functools import partial, partialmethod

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
    format = "%(asctime)s - %(filename)s:%(lineno)d - %(levelname)s - %(message)s"
    logging.basicConfig(level=level, filename=filename, format=format)

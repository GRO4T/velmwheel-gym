import logging


def init_logging(
    level: int = logging.DEBUG, filename: str = "./logs/velmwheel/default.log"
):
    format = "%(asctime)s - %(filename)s:%(lineno)d - %(levelname)s - %(message)s"
    logging.basicConfig(level=level, filename=filename, format=format)

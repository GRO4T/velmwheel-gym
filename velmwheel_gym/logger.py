import logging


def init_logging(
    level: int = logging.DEBUG, filename: str = "./logs/velmwheel/default.log"
):
    format = "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
    logging.basicConfig(level=level, filename=filename, format=format)

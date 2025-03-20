import logging


def configure_logger(level=logging.INFO, log_to_file=None):
    """configure the logger"""

    logger = logging.getLogger()
    logger.setLevel(level)
    formatter = logging.Formatter(
        "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
    )

    console_handler = logging.StreamHandler()
    console_handler.set_formatter(formatter)
    logger.addHandler(console_handler)

    if log_to_file:
        file_handler = logging.FileHandler(log_to_file)
        file_handler.setLevel(level)
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)

    return logger

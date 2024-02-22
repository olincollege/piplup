import logging
from pydrake.common import configure_logging


def init_logging():
    configure_logging()
    logging.getLogger().setLevel(logging.DEBUG)
    logging.getLogger("drake").setLevel(logging.INFO)

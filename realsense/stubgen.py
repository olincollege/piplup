import importlib
import os
from mypy import stubgen
import sys

if __name__ == "__main__":
    module = sys.argv[1]
    importlib.__import__(module)
    args = ["--output=.", f"--module={module}"]
    returncode = stubgen.main(args=args) or 0
    assert returncode == 0, returncode
    print(os.getcwd())

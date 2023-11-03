#!/bin/bash

find ../** -iname *.h -o -iname *.cc | xargs clang-format -i -style=file --verbose
black ../
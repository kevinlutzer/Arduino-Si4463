# .justfile

# Set the shell for running recipes
set shell := ["bash", "-cu"]

# Lint recipe using cargo clippy
lint:
    #!/bin/bash

    # Lint everything except the .pio directory
    find . \
    -path './.pio' -prune -o \
    \( \( -name \*.cpp -o -name \*.h -o -name \*.ino \) -a ! -iname \*soap\* \) \
    -print0 | xargs -0 -n 1 clang-format -i --verbose

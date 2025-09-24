# .justfile

# Set the shell for running recipes
set shell := ["bash", "-cu"]

# Lint recipe using cargo clippy
lint:
    #!/bin/bash

    # Lint everything except the .pio directory
    find . \
    -path './.pio' -prune -o \
    \( \( -name \*.cpp -o -name \*.h -o -name \*.ino \) -a ! -iname \*radio_config.h\* \) \
    -print0 | xargs -0 -n 1 clang-format -i --verbose

build example="receive_test":
    #!/bin/bash

    arduino-cli compile examples/{{example}} -b rp2040:rp2040:rpipico

upload example="receive_test":
    #!/bin/bash

    arduino-cli upload examples/{{example}} -b rp2040:rp2040:rpipico -p /dev/cu.usbmodem2101

monitor example="receive_test":
    #!/bin/bash

    arduino-cli monitor examples/{{example}} -b rp2040:rp2040:rpipico -p /dev/cu.usbmodem2101

lint-check:     
    #!/bin/bash

    # Check if files are properly formatted using clang-format
    # This follows the same pattern as the .justfile but checks instead of formatting
    echo "Checking code formatting..."
    formatting_errors=0
    
    # Use the same find pattern as in .justfile
    files=$(find . \
        -path './.pio' -prune -o \
        \( \( -name \*.cpp -o -name \*.h -o -name \*.ino \) -a ! -iname \*radio_config.h\* \) \
        -print)
        
    for file in $files; do
        if [ -f "$file" ]; then
        echo "Checking $file..."
        if ! diff -u "$file" <(clang-format "$file") > /dev/null; then
            echo "❌ File $file is not properly formatted"
            echo "Differences:"
            diff -u "$file" <(clang-format "$file") || true
            echo "To fix, run: clang-format -i \"$file\""
            formatting_errors=$((formatting_errors + 1))
        else
            echo "✅ File $file is properly formatted"
        fi
        fi
    done
        
    if [ $formatting_errors -gt 0 ]; then
        echo "❌ Found $formatting_errors formatting issues"
        echo "Run 'just lint' or 'clang-format -i' on the affected files to fix"
        exit 1
    else
        echo "✅ All files are properly formatted"
    fi
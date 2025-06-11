#!/bin/sh

cmake --build ./build

if [ $? -eq 0 ]; then
    echo "Compilation successful. Executable 'build/main' created."
else
    echo "Compilation failed. Please check the error messages above."
fi

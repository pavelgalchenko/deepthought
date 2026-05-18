#! /bin/sh
# Script to autorun things

EXEC_NAME="deepthought"

# search through the various places where there can be a deepthought executable
EXEC_PATH=""
if command -v $EXEC_NAME &> /dev/null; then
    EXEC_PATH="$EXEC_NAME"
elif [ -f "./$EXEC_NAME" ]; then
    EXEC_PATH="./$EXEC_NAME"
elif [ -f     "./build/Release/$EXEC_NAME" ]; then
    EXEC_PATH="./build/Release/$EXEC_NAME"
elif [ -f     "./build/RelWithDebInfo/$EXEC_NAME" ]; then
    EXEC_PATH="./build/RelWithDebInfo/$EXEC_NAME"
elif [ -f     "./build/Debug/$EXEC_NAME" ]; then
    EXEC_PATH="./build/Debug/$EXEC_NAME"
fi

echo "$EXEC_PATH"

$EXEC_PATH -d $1 -o $1/$2 -g $3

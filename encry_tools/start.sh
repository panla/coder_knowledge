#!/bin/sh

ORIGIN_DIR=$1
TARGET_DIR=$2
BUILD_NAME_FILE=$3

echo "ORIGIN_DIR = $ORIGIN_DIR"
echo "TAGRET_DIR = $TARGET_DIR"
echo "BUILD_NAME_FILE = $BUILD_NAME_FILE"

if [ ! -d "$ORIGIN_DIR" ]; then
    echo "ORIGIN_DIR not exists"
    exit 1
fi

if [ ! -f "$BUILD_NAME_FILE" ]; then
    echo "BUILD_NAME_FILE not exists"
fi

BASE_DIR=`pwd`
echo "BASE_DIR = $BASE_DIR"

# copy origin -> target
if [ ! -d $TARGET_DIR ]; then
    cp -r $ORIGIN_DIR $TARGET_DIR
else
    if [ $ORIGIN_DIR != $TARGET_DIR ]; then
        echo "ORIGIN_DIR != TARGET_DIR"
        rm -rf $TARGET_DIR
        cp -r $ORIGIN_DIR $TARGET_DIR
    else
        echo "ORIGIN_DIR == TARGET_DIR"
    fi
fi

cd $TARGET_DIR

IGNORE_DIRS=".idea .vscode .git tmp logs"
for _dir in $IGNORE_DIRS
do
    _f_dir="$TARGET_DIR/$_dir"
    if [ -d $_f_dir ]; then
        rm -rf $_f_dir
    fi
done

transform py files -> .so
cat $BUILD_NAME_FILE | while read LINE
do
    python "$BASE_DIR/build.py" -r $TARGET_DIR -n $LINE
done

# rename .so file name 也可以不用改名字
cd $BASE_DIR
python "$BASE_DIR/rename_so_name.py" -r $TARGET_DIR

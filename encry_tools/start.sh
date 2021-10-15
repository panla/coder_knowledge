ORIGIN_DIR=$1
TARGET_DIR=$2
BUILD_NAME_FILE=$3

BASE_DIR=`pwd`

# copy origin -> target
if [ ! -d $TARGET_DIR ]; then
    cp -r $ORIGIN_DIR $TARGET_DIR
else
    rm -rf $TARGET_DIR
    cp -r $ORIGIN_DIR $TARGET_DIR
fi

# transform py files -> .so
cat $BUILD_NAME_FILE | while read LINE
do
    cd "$TARGET_DIR/$LINE"
    python "$BASE_DIR/transform_py_so.py" -r $TARGET_DIR -n $LINE
done

# rename .so file name
cd $BASE_DIR
python "$BASE_DIR/rename_so_name.py" -r $TARGET_DIR

#/usr/bin/bash

conf_file=$1

ORIGIN_DIR=`cat $conf_file | jq -r '.ORIGIN_DIR'`
TARGET_DIR=`cat $conf_file | jq -r '.TARGET_DIR'`
TRANSFORM_SCRIPT_PATH=`cat $conf_file | jq -r '.TRANSFORM_SCRIPT_PATH'`
RENAME_SCRIPT_PATH=`cat $conf_file | jq -r '.RENAME_SCRIPT_PATH'`
NAME_LIS=`cat $conf_file | jq -r '.NAME_LIS[]'`

if [ ! -d $TARGET_DIR ]; then
    cp -r $ORIGIN_DIR $TARGET_DIR
else
    rm -rf $TARGET_DIR
    cp -r $ORIGIN_DIR $TARGET_DIR
fi

cd $TARGET_DIR

for name in $NAME_LIS;
do
    cd $name;
    python $TRANSFORM_SCRIPT_PATH -r $TARGET_DIR -n $name
    cd -;
done

cd $TARGET_DIR
python $RENAME_SCRIPT_PATH -r $TARGET_DIR

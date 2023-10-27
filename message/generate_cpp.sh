#!/bin/bash

MSG_DIR=$(pwd)

for f in $MSG_DIR/*.proto
do
  protoc -I=$MSG_DIR --cpp_out=$MSG_DIR $f
done

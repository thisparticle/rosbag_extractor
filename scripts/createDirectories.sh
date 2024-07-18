#!/bin/bash

folder=
if [[ $1 = -f ]]; then
    shift
    if [[ $1 != */ ]]; then
        folder=$1/
    else
        folder=$1
    fi
    shift
else
    echo "Not folder selected, default=result"
    folder=raw_data/
fi
### Script to create nested directories to store the output data from the bag.
# while [[ ! -z $1 ]] && [[ $1 != *__* ]]; do
#     mkdir -p ~/GEODE_RAWDATA/$folder$1
#     echo "Created ~/GEODE_RAWDATA/$folder$1"
#     shift
# done

while [[ ! -z $1 ]] && [[ $1 != *__* ]]; do
    mkdir -p $folder$1/
    echo "Created $folder$1/"
    shift
done


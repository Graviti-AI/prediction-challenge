#!/bin/bash

ITEM="simulator"
COMMIT_ID=$(git log --oneline -1 | awk '{print $1}')
DATE_TIME=$(date +%Y%m%d)
IMAGE_NAME="hub.graviti.cn/prediction-challenge/${ITEM}"

IMG_TAG=${DATE_TIME}-${COMMIT_ID}
IMAGE=${IMAGE_NAME}:${IMG_TAG}

bash make_image.sh ${IMAGE}
if [ $? != 0 ]; then
    echo 'failed to make simulator image'
    exit -1
fi

python3 deploy.py $* --image "${IMAGE}" --evaluation-id='8e404625-2927-414a-811b-767b8374420b'

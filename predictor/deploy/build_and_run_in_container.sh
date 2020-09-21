#!/bin/bash

ITEM="predictor"
COMMIT_ID=$(git log --oneline -1 | awk '{print $1}')
DATE_TIME=$(date +%N)
IMAGE_NAME="hub.graviti.cn/prediction-challenge"
IMG_TAG=${ITEM}-${COMMIT_ID}-${DATE_TIME}
IMAGE=${IMAGE_NAME}:${IMG_TAG}
sed -i "/image: \S*\:${ITEM}*/c\    image: ${IMAGE}" ./docker-compose.yaml

./make_image.sh ${IMAGE}

docker-compose up

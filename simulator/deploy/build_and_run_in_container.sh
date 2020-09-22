#!/bin/bash

ITEM="simulator"
COMMIT_ID=$(git log --oneline -1 | awk '{print $1}')
DATE_TIME=$(date +%Y%m%d)
IMAGE_NAME="hub.graviti.cn/prediction-challenge/${ITEM}"
IMG_TAG=${DATE_TIME}-${COMMIT_ID}
IMAGE=${IMAGE_NAME}:${IMG_TAG}

sed -i "/image: \S*${ITEM}\:*/c\    image: ${IMAGE}" ./docker-compose.yaml

./make_image.sh ${IMAGE}

docker-compose up

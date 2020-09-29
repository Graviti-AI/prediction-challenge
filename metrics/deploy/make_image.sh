#!/bin/bash
if [[ $#<1 ]]; then
    ITEM="perdictor"
    COMMIT_ID=$(git log --oneline -1 | awk '{print $1}')
    DATE_TIME=$(date +%Y%m%d)
    IMG_TAG=${DATE_TIME}-${COMMIT_ID}
else
  IMG_TAG=$1
fi

IMAGE_NAME="hub.graviti.cn/prediction-challenge/metrics:$IMG_TAG"

echo start to build "$IMAGE_NAME"


cd .. && sudo docker build -f deploy/Dockerfile -t "$IMAGE_NAME" .

echo build finished
echo to push it to server, run
echo sudo docker push -t "$IMAGE_NAME"

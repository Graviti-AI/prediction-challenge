#!/bin/bash
ITEM="simulator"
COMMIT_ID=$(git log --oneline -1 | awk '{print $1}')
DATE_TIME=$(date +%Y%m%d)
IMAGE_NAME="hub.graviti.cn/prediction-challenge"


echo "start build ${ITEM}..."
IMG_TAG=${ITEM}-${COMMIT_ID}-${DATE_TIME}
cd .. && docker build -f deploy/Dockerfile -t ${IMAGE_NAME}:${IMG_TAG} --no-cache .
docker rmi simulator-build-env
docker image prune -f --filter label=label-simulator-build-env=simulator-build-env

echo "done"
echo "to push the result, run"
echo "docker push ${IMAGE_NAME}:${IMG_TAG}"

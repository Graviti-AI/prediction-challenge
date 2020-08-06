#!/bin/bash
if [[ $#<1 ]]; then
    ITEM="simulator"
    COMMIT_ID=$(git log --oneline -1 | awk '{print $1}')
    DATE_TIME=$(date +%Y%m%d)
    IMAGE_NAME="hub.graviti.cn/prediction-challenge"

    echo "start build ${ITEM}..."
    IMG_TAG=${ITEM}-${COMMIT_ID}-${DATE_TIME}
    IMAGE=${IMAGE_NAME}:${IMG_TAG}
else
    IMAGE=$1
fi

cd ../.. && docker build -f simulator/deploy/Dockerfile -t ${IMAGE} --no-cache .
docker image prune -f --filter label=label-simulator-build-env=simulator-build-env

if [[ $#<1 ]]; then
    echo "done"
    echo "to push the result, run"
    echo "docker push ${IMAGE}"
fi

#!/bin/bash
if [[ $#<1 ]]; then
    ITEM="simulator"
    COMMIT_ID=$(git log --oneline -1 | awk '{print $1}')
    DATE_TIME=$(date +%Y%m%d)
    IMAGE_NAME="hub.graviti.cn/prediction-challenge/${ITEM}"

    echo "start build ${ITEM}..."
    IMG_TAG=${DATE_TIME}-${COMMIT_ID}
    IMAGE=${IMAGE_NAME}:${IMG_TAG}
else
    IMAGE=$1
fi

cd ../.. && docker build -f simulator/deploy/Dockerfile -t ${IMAGE} --no-cache .
docker image prune -f --filter label=label-simulator-build-env=simulator-build-env

sed -i "/image: \S*\:${ITEM}*/c\    image: ${IMAGE_NAME}:${IMG_TAG}" ./docker-compose.yaml

if [[ $#<1 ]]; then
    echo "done"
    echo "to push the result, run"
    echo "docker push ${IMAGE}"
fi

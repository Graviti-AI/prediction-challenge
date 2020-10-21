#!/bin/bash
if [[ $#<1 ]]; then
    ITEM="simulator"
    COMMIT_ID=$(git log --oneline -1 | awk '{print $1}')
    DATE_TIME=$(date +%Y%m%d)
    IMAGE_NAME="hub.graviti.cn/prediction-challenge/${ITEM}"

    IMG_TAG=${DATE_TIME}-${COMMIT_ID}
    IMAGE=${IMAGE_NAME}:${IMG_TAG}
    
    sed -i "/image: .*${ITEM}\:.*/c\    image: ${IMAGE}" ./docker-compose.yaml
else
    IMAGE=$1
fi


echo "start build ${IMAGE}..."

cd ../.. && docker build -f simulator/deploy/Dockerfile -t ${IMAGE} --no-cache .
docker image prune -f --filter label=label-simulator-build-env=simulator-build-env

if [[ $#<1 ]]; then
    echo "done"
    echo "to push the result, run"
    echo "docker push ${IMAGE}"
fi

#!/bin/bash
if [[ $#<1 ]]; then
    ITEM="perdictor"
    COMMIT_ID=$(git log --oneline -1 | awk '{print $1}')
    DATE_TIME=$(date +%Y%m%d)
    IMAGE_NAME="hub.graviti.cn/prediction-challenge"

    echo "start build ${ITEM}..."
    IMG_TAG=${ITEM}-${COMMIT_ID}-${DATE_TIME}
    IMAGE=${IMAGE_NAME}:${IMG_TAG}
else
    IMAGE=$1
fi

cd ..
docker build -f deploy/Dockerfile -t ${IMAGE} .

sed -i "/image: \S*\:${ITEM}*/c\    image: ${IMAGE_NAME}:${IMG_TAG}" ./docker-compose.yaml

echo build complete, to push the result, run:
echo sudo docker push ${IMAGE}

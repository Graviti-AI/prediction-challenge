#!/bin/bash
if [[ $#<1 ]]; then
    ITEM="perdictor"
    COMMIT_ID=$(git log --oneline -1 | awk '{print $1}')
    DATE_TIME=$(date +%Y%m%d)
    IMAGE_NAME="${ITEM}"

    echo "start build ${ITEM}..."
    IMG_TAG=${DATE_TIME}-${COMMIT_ID}
    IMAGE=${IMAGE_NAME}:${IMG_TAG}
    
else
    IMAGE=$1
fi

sed -i "/image: .*/c\    image: ${IMAGE}" ./docker-compose.yaml

cd ..
docker build -f deploy/Dockerfile -t ${IMAGE} .


echo build ${IMAGE} completed
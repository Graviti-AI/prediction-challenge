#!/bin/bash
if [[ $#<1 ]]; then
    ITEM="perdictor"
    COMMIT_ID=$(git log --oneline -1 | awk '{print $1}')
    DATE_TIME=$(date +%Y%m%d)
    IMAGE_NAME="hub.graviti.cn/prediction-challenge/${ITEM}"

    echo "start build ${ITEM}..."
    IMG_TAG=${DATE_TIME}-${COMMIT_ID}
    IMAGE=${IMAGE_NAME}:${IMG_TAG}
    
    sed -i "/image: .*${ITEM}\:.*/c\    image: ${IMAGE}" ./docker-compose.yaml
else
    IMAGE=$1
fi

cd ..
docker build -f deploy/Dockerfile -t ${IMAGE} .


echo build complete, to push the result, run:
echo sudo docker push ${IMAGE}

echo tag predictor to user repo
USER_REPO_IMAGE=registry.graviti.cn/1b7683bc81b14972099408f83c76fb71/predictor:${IMG_TAG}
sudo docker tag ${IMAGE} ${USER_REPO_IMAGE}
echo to push predictor to user repo, run
echo sudo docker push ${USER_REPO_IMAGE}


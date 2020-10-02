#!/bin/bash
if [[ $#<1 ]]; then
  VERSION="0.1"
else
  VERSION=$1
fi

IMAGE_NAME="registry.graviti.cn/model-evaluation/metrics-berkeley-simulation:$VERSION"

echo start to build "$IMAGE_NAME"

sudo docker build -t "$IMAGE_NAME" .

echo build finished
echo to push it to server, run
echo sudo docker push -t "$IMAGE_NAME"

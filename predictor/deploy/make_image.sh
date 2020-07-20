#!/bin/bash
cd ..
docker build -f deploy/Dockerfile -t $1 .

echo build complete, to push the rusult, run:
echo sudo docker push $1

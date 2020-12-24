FROM hub.graviti.cn/prediction-challenge/predictor-base-torch-v1.0

RUN mkdir /root/predictor

COPY ./*.py             /root/predictor/
COPY ./predictors/       /root/predictor/predictors/
COPY requirement.txt    /root/predictor/

RUN pip3 install -r /root/predictor/requirement.txt

WORKDIR /root/predictor
CMD ["python3", "main.py", "-s", "127.0.0.1", "-p", "50051"]

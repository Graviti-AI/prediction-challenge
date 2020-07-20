FROM python:3.7-buster

RUN mkdir /root/predictor

COPY ./*.py             /root/predictor/
COPY ./predictor/       /root/predictor/predictor/
COPY requirement.txt    /root/predictor/

RUN pip3 install -r /root/predictor/requirement.txt

WORKDIR /root/predictor
CMD ["python3", "main.py", "-s", "127.0.0.1", "-p", "50051"]

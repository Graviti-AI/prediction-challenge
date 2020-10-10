FROM hub.graviti.cn/prediction-challenge/metrics-base:0.1

RUN mkdir -p /prediction-challenge/metrics/datacenter

COPY ./*.py             /prediction-challenge/metrics/
COPY ./utils/*.py       /prediction-challenge/metrics/utils/
COPY ./datacenter/*.py  /prediction-challenge/metrics/datacenter/

COPY ./requirements.txt requirements.txt
RUN pip3 install -r requirements.txt 
RUN rm requirements.txt

WORKDIR /prediction-challenge/metrics/

CMD [ "python3", "main.py"]

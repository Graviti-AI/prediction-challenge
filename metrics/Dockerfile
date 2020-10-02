FROM registry.graviti.cn/model-evaluation/metrics-build-env:v0.4

RUN mkdir -p /usr/local/model-evaluation/simulation-metrics/datacenter

COPY ./*.py             /usr/local/model-evaluation/simulation-metrics/
COPY ./datacenter/*.py  /usr/local/model-evaluation/simulation-metrics/datacenter/

COPY ./requirements.txt requirements.txt
RUN pip3 install -i https://pypi.tuna.tsinghua.edu.cn/simple -r requirements.txt
RUN rm requirements.txt

WORKDIR /usr/local/model-evaluation/simulation-metrics/

CMD [ "python3", "main.py"]

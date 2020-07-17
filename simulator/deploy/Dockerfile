FROM hub.graviti.cn/prediction-challenge/simulator-build-base:0.1 AS simulator-build-env
LABEL label-simulator-build-env=simulator-build-env

RUN mkdir /prediction-challenge
COPY ./ /prediction-challenge/simulator

WORKDIR /prediction-challenge/simulator
RUN make

FROM simulator-base:1.0
RUN mkdir -p /prediction-challenge/simulator

COPY --from=simulator-build-env /prediction-challenge/simulator/simulator /prediction-challenge/simulator

WORKDIR /prediction-challenge/simulator
cmd ["/prediction-challenge/simulator/simulator"]

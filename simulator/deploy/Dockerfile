FROM hub.graviti.cn/prediction-challenge/simulator-base:1.0 AS simulator-build-env
LABEL label-simulator-build-env=simulator-build-env

RUN mkdir -p /prediction-challenge/simulator_src/build
COPY ./. /prediction-challenge/simulator_src/
RUN cd /prediction-challenge/simulator_src/build && cmake .. && make

FROM hub.graviti.cn/prediction-challenge/simulator-base:1.0
RUN mkdir -p /prediction-challenge/
COPY --from=simulator-build-env /prediction-challenge/simulator_src/build/simulator /prediction-challenge/

WORKDIR /prediction-challenge
cmd ["/prediction-challenge/simulator", "-p", "50051"]

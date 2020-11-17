FROM hub.graviti.cn/prediction-challenge/simulator-base:1.1 AS simulator-build-env
LABEL label-simulator-build-env=simulator-build-env

RUN mkdir -p /prediction-challenge/simulator_src/build
COPY ./simulator/. /prediction-challenge/simulator_src/
COPY ./proto/. /prediction-challenge/simulator_src/proto/

COPY ./simulator/libsim.a /usr/lib/
COPY ./simulator/libjsoncpp.a /usr/lib/

WORKDIR /prediction-challenge/simulator_src/

RUN protoc -I ./proto/ --grpc_out=./service/proto/ --plugin=protoc-gen-grpc=`which grpc_cpp_plugin` ./proto/simulator.proto
RUN protoc -I ./proto/ --cpp_out=./service/proto/ ./proto/simulator.proto
RUN cd /prediction-challenge/simulator_src/build && cmake .. && make -j4

FROM hub.graviti.cn/prediction-challenge/simulator-base:1.1
RUN mkdir -p /prediction-challenge/
RUN mkdir -p /prediction-challenge/conf/
RUN mkdir -p /core/my_impl/Maps/

# copy configuration files
COPY ./simulator/conf/. /prediction-challenge/conf/

# copy map data
COPY ./simulator/core/my_impl/Maps/only_positive_xy/. /core/my_impl/Maps/only_positive_xy/.
COPY ./simulator/core/my_impl/Maps/with_negative_xy/. /core/my_impl/Maps/with_negative_xy/.

# copy CSV data
RUN mkdir -p /core/my_impl/CSV/
COPY ./simulator/core/my_impl/CSV/. /core/my_impl/CSV/.

# copy binary
COPY --from=simulator-build-env /prediction-challenge/simulator_src/build/simulator /prediction-challenge/
COPY --from=simulator-build-env /prediction-challenge/simulator_src/build/core/my_impl/Lanelet2Lib/liblanelet2.so /prediction-challenge/

WORKDIR /prediction-challenge

ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/prediction-challenge

CMD ["/prediction-challenge/simulator", "-p", "50051", "-c", "/prediction-challenge/conf/config.txt", "-l", "/mnt/simulator_logs"]

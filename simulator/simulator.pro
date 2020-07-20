QT -= core gui

CONFIG += c++14 console
CONFIG -= app_bundle
LIBS += -L/usr/local/lib -lprotobuf -lgrpc++

INCLUDEPATH += .

SOURCES += \
        core/impl/default_simulator_impl.cpp \
        core/simulator.cpp \
        main.cpp \
        service/impl/service_impl.cpp \
        service/proto/simulator.grpc.pb.cc \
        service/proto/simulator.pb.cc \
        service/service.cpp

HEADERS += \
    core/impl/default_simulator_impl.h \
    core/simulator.h \
    core/trajectory.h \
    service/impl/service_impl.h \
    service/proto/simulator.grpc.pb.h \
    service/proto/simulator.pb.h \
    service/service.h

DISTFILES += \
    README.md \
    deploy/Dockerfile \
    deploy/build_and_run_in_container.sh \
    deploy/docker-compose.yaml \
    deploy/make_image.sh \
    depoly/Dockerfile \
    depoly/make_image.sh \
    service/proto/simulator.proto \
    simulator

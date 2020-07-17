QT -= core gui

CONFIG += c++14 console
CONFIG -= app_bundle
LIBS += -lgrpc++ -lprotobuf

INCLUDEPATH += .

SOURCES += \
        core/impl/simulator_impl.cpp \
        core/simulator.cpp \
        main.cpp \
        service/impl/service_impl.cpp \
        service/proto/simulator.grpc.pb.cc \
        service/proto/simulator.pb.cc \
        service/service.cpp

HEADERS += \
    core/impl/simulator_impl.h \
    core/simulator.h \
    core/trajectory.h \
    service/impl/service_impl.h \
    service/proto/simulator.grpc.pb.h \
    service/proto/simulator.pb.h \
    service/service.h

DISTFILES += \
    depoly/Dockerfile \
    depoly/make_image.sh \
    service/proto/simulator.proto

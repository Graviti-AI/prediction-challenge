QT -= core gui

CONFIG += c++14 console
CONFIG -= app_bundle
LIBS += -L/usr/local/lib -lprotobuf -lgrpc++ -lgrpc++_reflection
#LIBS += -lgrpc_plugin_support -laddress_sorting
#LIBS += -lgrpcpp_channelz -lgrpc_unsecure
#LIBS += -lgrpc++_alts -lgrpc++_error_details -lgrpc++_unsecure -lgrpc++ -lgrpc -lgrpc++_reflection  -lupb  -lgpr
#LIBS += -lgrpc++ -labsl_bad_optional_access -labsl_str_format_internal -labsl_time -labsl_time_zone -labsl_civil_time -labsl_strings -labsl_strings_internal -labsl_throw_delegate -labsl_int128 -labsl_base -labsl_spinlock_wait -labsl_raw_logging_internal -labsl_log_severity -labsl_dynamic_annotations

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
    deploy/make_image.sh \
    depoly/Dockerfile \
    depoly/make_image.sh \
    service/proto/simulator.proto \
    simulator

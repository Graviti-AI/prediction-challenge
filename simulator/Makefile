MAKEFILE      = Makefile

CC            = gcc
CXX           = g++
DEFINES       = -DQT_NO_DEBUG
CFLAGS        = -pipe -O2 -Wall -W -D_REENTRANT -fPIC $(DEFINES)
CXXFLAGS      = -pipe -O2 -std=gnu++1y -Wall -W -D_REENTRANT -fPIC $(DEFINES)
INCPATH       = -I.
DEL_FILE      = rm -f
CHK_DIR_EXISTS= test -d
MKDIR         = mkdir -p
COPY          = cp -f
COPY_FILE     = cp -f
COPY_DIR      = cp -f -R
INSTALL_FILE  = install -m 644 -p
INSTALL_PROGRAM = install -m 755 -p
INSTALL_DIR   = cp -f -R
DEL_FILE      = rm -f
SYMLINK       = ln -f -s
DEL_DIR       = rmdir
MOVE          = mv -f
TAR           = tar -cf
COMPRESS      = gzip -9f
DISTNAME      = simulator1.0.0
LINK          = g++
LFLAGS        = -Wl,-O1
LIBS          = $(SUBLIBS) -lgrpc++ -lprotobuf -lpthread 
AR            = ar cqs
RANLIB        = 
SED           = sed
STRIP         = strip

OBJECTS_DIR   = ./.tmp

SOURCES       = core/impl/simulator_impl.cpp \
		core/simulator.cpp \
		main.cpp \
		service/impl/service_impl.cpp \
		service/proto/simulator.grpc.pb.cc \
		service/proto/simulator.pb.cc \
		service/service.cpp 
OBJECTS       = simulator_impl.o \
		simulator.o \
		main.o \
		service_impl.o \
		simulator.grpc.pb.o \
		simulator.pb.o \
		service.o
DIST          = service/proto/simulator.proto \
		simulator.pro core/impl/simulator_impl.h \
		core/simulator.h \
		core/trajectory.h \
		service/impl/service_impl.h \
		service/proto/simulator.grpc.pb.h \
		service/proto/simulator.pb.h \
		service/service.h core/impl/simulator_impl.cpp \
		core/simulator.cpp \
		main.cpp \
		service/impl/service_impl.cpp \
		service/proto/simulator.grpc.pb.cc \
		service/proto/simulator.pb.cc \
		service/service.cpp
DESTDIR       = 
TARGET        = simulator


first: all
####### Build rules

$(TARGET):  $(OBJECTS)  
	$(LINK) $(LFLAGS) -o $(TARGET) $(OBJECTS) $(OBJCOMP) $(LIBS)

all: $(TARGET)

dist: distdir FORCE
	(cd `dirname $(DISTDIR)` && $(TAR) $(DISTNAME).tar $(DISTNAME) && $(COMPRESS) $(DISTNAME).tar) && $(MOVE) `dirname $(DISTDIR)`/$(DISTNAME).tar.gz . && $(DEL_FILE) -r $(DISTDIR)

distdir: FORCE
	@test -d $(DISTDIR) || mkdir -p $(DISTDIR)
	$(COPY_FILE) --parents $(DIST) $(DISTDIR)/


clean: compiler_clean 
	-$(DEL_FILE) $(OBJECTS)
	-$(DEL_FILE) *~ core *.core


distclean: clean 
	-$(DEL_FILE) $(TARGET) 
	-$(DEL_FILE) .qmake.stash
	-$(DEL_FILE) Makefile


####### Sub-libraries

check: first

benchmark: first

compiler_yacc_decl_make_all:
compiler_yacc_decl_clean:
compiler_yacc_impl_make_all:
compiler_yacc_impl_clean:
compiler_lex_make_all:
compiler_lex_clean:
compiler_clean: 

####### Compile

simulator_impl.o: core/impl/simulator_impl.cpp core/impl/simulator_impl.h \
		core/simulator.h \
		core/trajectory.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o simulator_impl.o core/impl/simulator_impl.cpp

simulator.o: core/simulator.cpp core/simulator.h \
		core/trajectory.h \
		core/impl/simulator_impl.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o simulator.o core/simulator.cpp

main.o: main.cpp core/simulator.h \
		core/trajectory.h \
		service/service.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o main.o main.cpp

service_impl.o: service/impl/service_impl.cpp service/impl/service_impl.h \
		service/proto/simulator.grpc.pb.h \
		service/proto/simulator.pb.h \
		core/simulator.h \
		core/trajectory.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o service_impl.o service/impl/service_impl.cpp

simulator.grpc.pb.o: service/proto/simulator.grpc.pb.cc service/proto/simulator.pb.h \
		service/proto/simulator.grpc.pb.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o simulator.grpc.pb.o service/proto/simulator.grpc.pb.cc

simulator.pb.o: service/proto/simulator.pb.cc service/proto/simulator.pb.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o simulator.pb.o service/proto/simulator.pb.cc

service.o: service/service.cpp service/service.h \
		service/impl/service_impl.h \
		service/proto/simulator.grpc.pb.h \
		service/proto/simulator.pb.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o service.o service/service.cpp

####### Install

install:  FORCE

uninstall:  FORCE

FORCE:


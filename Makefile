
OR_TOOLS_TOP=../or-tools_N

FLANN_MASTER_TOP = ../flann

CCC=g++ -fPIC -std=c++11 -fwrapv -O4 -DNDEBUG
CFLAGS=-I $(OR_TOOLS_TOP)/include -I $(FLANN_MASTER_TOP)/src/cpp #. -isystem./cut/


.PHONY: all local_clean

all: $(EXE)

%.pb.cc: %.proto
	$(OR_TOOLS_TOP)/bin/protoc --cpp_out . $<
	$(OR_TOOLS_TOP)/bin/protoc --ruby_out . $<

problem.pb.h: problem.pb.cc

%.o: %.cc %.h
	$(CCC) $(CFLAGS) -c $< -o $@

clustering.o: ./clustering.cc ./problem.pb.h
	$(CCC) $(CFLAGS) -c ./clustering.cc -o clustering.o

clustering: $(ROUTING_DEPS) clustering.o  problem.pb.o
	$(CCC) $(CFLAGS) -g clustering.o problem.pb.o   \
	-L $(FLANN_MASTER_TOP)/build/lib -lflann \
	-L $(OR_TOOLS_TOP)/lib -lprotobuf \
	-Wl,-rpath,$(OR_TOOLS_TOP)/lib \
	-Wl,-rpath,$(FLANN_MASTER_TOP)/build/lib  \
	-o clustering

local_clean:
	rm -f *.pb.cc *.pb.h *.o

mrproper: local_clean
	rm -f clustering

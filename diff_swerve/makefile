
CXX = g++
CXXFLAGS = -Wall -pthread
LDLIBS = -lpigpio -lrt -levdev

SRCS = main.cpp n20_motor.cpp xbox_controller.cpp logger.cpp util.cpp MiniPID.cpp
OBJS = $(subst .cpp,.o,$(SRCS))

.PHONY = all run plot clean

all: output run

output: $(OBJS)
	$(CXX) $(CXXFLAGS) -o output $(OBJS) $(LDLIBS)

$(OBJS):
n20_motor.cpp : n20_motor.h
xbox_controller.cpp: xbox_controller.h
logger.cpp : logger.h
util.cpp : util.h
MiniPID.cpp : MiniPID.h

run:
	@ sudo -E ./output

plot:
	@ make output
	@ PLOT=true sudo -E ./output | feedgnuplot --lines --domain --term 'dumb ansirgb 100, 30' --stream 0.1 --xlen 5 --set 'autoscale y' --unset 'border'

clean:
	@ rm -f $(OBJS)

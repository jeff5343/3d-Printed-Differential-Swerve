
CXX = g++
CXXFLAGS = -Wall -pthread
LDLIBS = -lpigpio -lrt -levdev

SRCS = src/main.cpp src/components/n20_motor.cpp src/componentsxbox_controller.cpp src/utils/logger.cpp src/utils/util.cpp libs/MiniPID.cpp
OBJS = $(subst .cpp,.o,$(SRCS))

.PHONY = all run plot clean

all: output run

output: $(OBJS)
	$(CXX) $(CXXFLAGS) -o output $(OBJS) $(LDLIBS)

$(OBJS):
n20_motor.cpp : libs/n20_motor.h
xbox_controller.cpp: src/components/xbox_controller.h
logger.cpp : src/utils/logger.h
util.cpp : src/utils/util.h
MiniPID.cpp : libs/MiniPID.h

run:
	@ sudo -E ./output

plot:
	@ make output
	@ PLOT=true sudo -E ./output | feedgnuplot --lines --domain --term 'dumb ansirgb 100, 30' --stream 0.1 --xlen 5 --set 'autoscale y' --unset 'border'

clean:
	@ rm -f $(OBJS)

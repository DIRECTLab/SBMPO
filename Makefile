# Compiler and Flags
CPP=g++
AR=ar

ifeq ($(target),release)
	FEATURE=$(feature)
	CFLAGS=--std=c++11 -O2 -Wall -pedantic -w -fPIC
else
ifeq ($(target),debug)
	# Output visualizer information and log output to debug log file
	FEATURE=$(feature)
	CFLAGS=--std=c++11 -O2 -Wall -pedantic -g3 -w -fPIC -DDEBUG_BUILD
else
	FEATURE=$(feature)
	target=dev
	CFLAGS=--std=c++11 -O2 -Wall -pedantic -g3 -w -fPIC
endif
endif


# Directories
APP=./app
SRCS=./lib/src
INCLUDE=./lib/include
# adding Target allows us to avoid issues with mixing different build target's
# .o files
BIN=./bin/$(target)


# Binary arguments
RESULTS=./results/results.json
ifeq ($(example),kinematic)
	CONFIG=./resources/kinematic/config.json
else
ifeq ($(example),spot)
 	CONFIG=./resources/spot/config.json
else
	example=1D
	CONFIG=./resources/1D/config.json
endif
endif

# Other
LOG=results/debug_log
DOCCFG=doxygen_config

# Object files
OBJS=$(BIN)/collision.o \
	$(BIN)/controlConstraints.o \
	$(BIN)/sampling.o \
	$(BIN)/energyModel.o \
	$(BIN)/1D.o \
	$(BIN)/config.o \
	$(BIN)/planner.o

#-------------------------------------------------------------------------------
# Main rules

.PHONY:all
all: lib $(BIN)/sbmpo

run: $(BIN)/sbmpo
ifneq ($(example),kinematic)
ifneq ($(example),1D)
ifneq ($(example),spot)
	@echo "Invalid config."
	@echo "Please use one of"
	@echo "    make example=1D"
	@echo "    make example=kinematic"
	@echo "    make example=spot"
	@exit 1
endif
endif
endif
	@echo ".... \033[01;39mRunning '$(example)' example\033[0m ...."
	$(BIN)/sbmpo $(CONFIG) --output=$(RESULTS)

lib: $(BIN)/sbmpo.so $(BIN)/sbmpo.a

#-------------------------------------------------------------------------------
# Dummy rules

.PHONY:doc
doc:
	doxygen $(DOCCFG)

.PHONY:log
log:
	cat $(LOG)

.PHONY:examples
examples:
	@echo "\033[01;39m1D\033[0m ... run 1-dimensional double integrator model"
	@echo "\033[01;39mkinematic\033[0m ... run kinematic skid-steered robot model"
	@echo "\033[01;39mspot\033[0m ... run spot robot model"

.PHONY:targets
targets:
	@echo "\033[01;39mdev\033[0m ... optimized build with gdb debug symbols"
	@echo "\033[01;39mdebug\033[0m ... enable debug log and other debug support"
	@echo "\033[01;39mrelease\033[0m ... fully optimized release profile"

#------------------------------------------------------------------------------
# commands to compile the executable.

$(BIN)/sbmpo.a: $(OBJS)
	@echo ".... \033[01;39mStatic library for '$(target)' target\033[0m ...."
	$(AR) rvsU $@ $^

$(BIN)/sbmpo.so: $(OBJS)
	@echo ".... \033[01;39mDynamic library for '$(target)' target\033[0m ...."
	$(CPP) -shared  -Wl,-soname,$@ -o $@ $^

$(BIN)/sbmpo: $(APP)/main.cpp $(BIN)/sbmpo.a
	@echo ".... \033[01;39mBuild exectuable for '$(target)' target\033[0m ...."
	$(CPP) $(CFLAGS) -o $@ -static $^ -I $(INCLUDE)

#------------------------------------------------------------------------------
# Define object files and their dependencies

$(BIN)/collision.o : \
		$(SRCS)/collision.cpp \
		$(INCLUDE)/collision.h
	$(CPP) -I $(INCLUDE) -c $(CFLAGS) $(FEATURE) $(SRCS)/collision.cpp -o $@

$(BIN)/controlConstraints.o : \
		$(SRCS)/controlConstraints.cpp \
		$(INCLUDE)/controlConstraints.h
	$(CPP) -I $(INCLUDE) -c $(CFLAGS) $(FEATURE) $(SRCS)/controlConstraints.cpp -o $@

$(BIN)/sampling.o : \
		$(SRCS)/sampling.cpp \
		$(INCLUDE)/sampling.h \
		$(INCLUDE)/definitions.h
	$(CPP) -I $(INCLUDE) -c $(CFLAGS) $(FEATURE) $(SRCS)/sampling.cpp -o $@

$(BIN)/1D.o : \
		$(INCLUDE)/model/1D.h \
		$(SRCS)/model/1D.cpp \
		$(INCLUDE)/definitions.h \
		$(INCLUDE)/model.h
	$(CPP) -I $(INCLUDE) -c $(CFLAGS) $(FEATURE) $(SRCS)/model/1D.cpp -o $@

$(BIN)/energyModel.o : \
		$(INCLUDE)/model/energy.h \
		$(SRCS)/model/energy.cpp \
		$(INCLUDE)/definitions.h \
		$(INCLUDE)/model.h
	$(CPP) -I $(INCLUDE) -c $(CFLAGS) $(FEATURE) $(SRCS)/model/energy.cpp -o $@

$(BIN)/config.o : \
		$(SRCS)/config.cpp \
		$(INCLUDE)/config.h \
		$(INCLUDE)/definitions.h
	$(CPP) -I $(INCLUDE) -c $(CFLAGS) $(FEATURE) $(SRCS)/config.cpp -o $@

$(BIN)/planner.o : $(SRCS)/sbmpo.cpp \
		$(INCLUDE)/sbmpo.h \
		$(INCLUDE)/definitions.h
	$(CPP) -I $(INCLUDE) -c $(CFLAGS) $(FEATURE) $(SRCS)/sbmpo.cpp -o $@

#------------------------------------------------------------------------------
# Clean Commands

.PHONY: clean
clean:
	rm -rf $(BIN)/*.o $(BIN)/sbmpo ./results/*.json ./results/*.txt docs
	@find $(BIN) || mkdir $(BIN)
	@echo cleaned.

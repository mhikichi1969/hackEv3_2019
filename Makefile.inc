mkfile_path := $(dir $(lastword $(MAKEFILE_LIST)))

APPL_COBJS += balancer.o balancer_param.o

APPL_CXXOBJS += \
HBTtask.o \
util.o \
HPolling.o \
Odometry.o \
Tracer.o \
ArmControl.o \
SceneWalker.o \
HCalibrator.o \
HPID.o \
HColorSensor.o \
HLowPassFilter.o \
Starter.o \
SDFile.o \
SimpleWalker.o \
LineTracer.o \
Turn.o \
StraightWalker.o \
SpeedSection.o \
CompositeSection.o \
Judge.o \
SectionJudge.o \
Section.o \
Runner.o \
BlockPlace.o \
CrossCircle.o \
SectionCreate.o \
Area.o \
BlockCircle.o \
BlackLine.o \
Block.o \
Distance.o \
BlockBingo.o \
CarryBlock.o \
SpeedControl.o \
Timer.o \
DeviceError.o \
VirtualTracer.o


SRCLANG := c++

ifdef CONFIG_EV3RT_APPLICATION

# Include libraries
include $(EV3RT_SDK_LIB_DIR)/libcpp-ev3/Makefile

endif

APPL_DIR += $(mkfile_path)app $(mkfile_path)device  $(mkfile_path)judge  $(mkfile_path)section  $(mkfile_path)walker  $(mkfile_path)bingo 

INCLUDES += \
	-I$(mkfile_path)app \
	-I$(mkfile_path)device \
	-I$(mkfile_path)judge \
	-I$(mkfile_path)section \
	-I$(mkfile_path)walker \
	-I$(mkfile_path)runner \
	-I$(mkfile_path)bingo \


COPTS += -fno-use-cxa-atexit

mkfile_path := $(dir $(lastword $(MAKEFILE_LIST)))

APPL_CXXOBJS += \
	util.o \
	HPolling.o \
	HColorSensor.o \
	HLowPassFilter.o \
	Odometry.o \
	Judge.o \
	SectionJudge.o \
	SceneWalker.o \
	HCalibrator.o \
	ArmControl.o \
	HPID.o \
	SimpleWalker.o \
	SpeedControl.o \
	LineTracer.o \
	Starter.o \
	Section.o \
	SpeedSection.o	\
	Turn.o \
	StraightWalker.o \
	VirtualTracer.o 

	


SRCLANG := c++

ifdef CONFIG_EV3RT_APPLICATION

# Include libraries
include $(EV3RT_SDK_LIB_DIR)/libcpp-ev3/Makefile

endif

APPL_DIRS += $(mkfile_path)app $(mkfile_path)device  $(mkfile_path)judge  $(mkfile_path)section  $(mkfile_path)walker  $(mkfile_path)bingo 

INCLUDES += \
	-I$(mkfile_path)app \
	-I$(mkfile_path)device \
	-I$(mkfile_path)judge \
	-I$(mkfile_path)section \
	-I$(mkfile_path)walker \
	-I$(mkfile_path)runner \
	-I$(mkfile_path)bingo \

COPTS += -std=c++11

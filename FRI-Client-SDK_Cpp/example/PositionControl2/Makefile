BASE_DIR	= ../..
include $(BASE_DIR)/build/GNUMake/paths.mak
include $(BASE_DIR)/build/GNUMake/$(TOOLS_MAK)



SOURCES 		= PositionControlApp.cpp \
			 	  PositionControlClient.cpp

TARGET 		= PositionControl

CXXFLAGS 	+= 
LDFLAGS 		+= $(LIB_DIR)/libFRIClient.a
INCLUDE = -I/usr/include/

include $(BASE_DIR)/build/GNUMake/rules_examples.mak


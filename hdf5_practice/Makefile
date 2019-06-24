# Edit the following variables as needed
HDF_INSTALL = /usr/local/hdf5
# EXTLIB      = -L$(HDF_INSTALL)/lib64		# KUKA Computer
EXTLIB		= -L$(HDF_INSTALL)/lib 			# Tanner's Laptop
# HDFLIB      = $(HDF_INSTALL)/lib64		# KUKA Computer
HDFLIB		= $(HDF_INSTALL)/lib			# Tanner's Laptop
CC          = g++
CFLAGS      = -std=c++11 -pthread
LIB         = -lz -lm -ldl #-lsz

TARGET = practice

INCLUDE   = -I$(HDF_INSTALL)/include
LIBSHDF   = $(EXTLIB) -lhdf5_cpp \
                      -lhdf5

all: $(TARGET)

$(TARGET): $(TARGET).o
	$(CC) $(CFLAGS) -o $@ $^ $(LIBSHDF) $(LIB)

$(TARGET).o: $(TARGET).cpp
	$(CC) $(CFLAGS) -o $@ -c $^ $(INCLUDE)

clean:
	rm -f *.h5 *.o \
			$(TARGET)

.SUFFIXES:.o.c

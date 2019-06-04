# Edit the following variables as needed
HDF_INSTALL = /usr/local/hdf5
EXTLIB      = -L$(HDF_INSTALL)/lib64
HDFLIB      = $(HDF_INSTALL)/lib64
CC          = g++
CFLAGS      =
LIB         = -lz -lm -ldl #-lsz

TARGET = h5_practice

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

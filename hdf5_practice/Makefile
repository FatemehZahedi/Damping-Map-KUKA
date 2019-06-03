# Edit the following variables as needed
HDF_INSTALL = /usr/local/hdf5
EXTLIB      = -L$(HDF_INSTALL)/lib
HDFLIB      = $(HDF_INSTALL)/lib
CC          = g++
CFLAGS      =
LIB         = -lsz -lz -lm -ldl

TARGET = h5_practice

INCLUDE   = -I$(HDF_INSTALL)/include
LIBSHDF   = $(EXTLIB) -lhdf5_cpp \
                      -lhdf5
LIBSHDF_CPP = -L$(HDFLIB)/libhdf5_cpp.a $(HDFLIB)/libhdf5_hl_cpp.a

all: $(TARGET)

$(TARGET): $(TARGET).o
	$(CC) $(CFLAGS) -o $@ $^ $(LIBSHDF) $(LIB)

$(TARGET).o: $(TARGET).cpp
	$(CC) $(CFLAGS) -o $@ -c $^ $(INCLUDE)

clean:
	rm -f *.h5 *.o \
			$(TARGET)

.SUFFIXES:.o.c

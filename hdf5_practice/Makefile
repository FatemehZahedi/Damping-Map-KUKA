# CXX = h5c++
#
# all: create
#
# create: create.cpp
# 	$(CXX) -o $@ create.cpp
#
# clean:
# 	rm -rf create \
# 	       *.h5 \
#  		   *.o


# Edit the following variables as needed
HDF_INSTALL = /usr/local/hdf5
EXTLIB      = -L$(HDF_INSTALL)/lib
HDFLIB      = $(HDF_INSTALL)/lib
CC          = g++
# CFLAGS      = -fPIC
CFLAGS      =
LIB         = -lsz -lz -lm -ldl

INCLUDE   = -I$(HDF_INSTALL)/include
# LIBSHDF   = $(EXTLIB) $(HDF_INSTALL)/lib/libhdf5_cpp.so $(HDF_INSTALL)/lib/libhdf5.so.103.1.0 $(HDF_INSTALL)/lib/libhdf5_cpp.so.103.1.0
LIBSHDF   = $(EXTLIB) -lhdf5_cpp \
                      -lhdf5 \
					  # -lhdf5_hl \
					  # -lhdf5_hl_cpp \
					  # -L$(HDFLIB)/libhdf5.la\
					  # -L$(HDFLIB)/libhdf5_cpp.la\
					  # -L$(HDFLIB)/libhdf5_hl.la\
					  # -L$(HDFLIB)/libhdf5_hl_cpp.la
LIBSHDF_CPP = -L$(HDFLIB)/libhdf5_cpp.a $(HDFLIB)/libhdf5_hl_cpp.a

all: create

create: create.o
	$(CC) $(CFLAGS) -o $@ create.o $(LIBSHDF) $(LIB)

create.o: create.cpp
	$(CC) $(CFLAGS) -o $@ -c create.cpp $(INCLUDE)

clean:
	rm -f *.h5 *.o \
			create

.SUFFIXES:.o.c

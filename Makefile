SRCS= $(wildcard *.cpp)
LIBNAME= include
INCDIR= .
DEFINES= 
include Makefile.config
include Makefile.template

MAKE = make -j 2
SUB_LIBS = . planning optimization statistics geometry meshing spline image robotics GLdraw math math3d camera  utils

default: $(OBJS) KrisLibrary

PQP:
	cd geometry/PQP; $(MAKE)

all: $(OBJS) PQP
	 cd utils; $(MAKE)
	 cd math; $(MAKE)
	 cd math3d; $(MAKE)
	 cd camera; $(MAKE)
	 cd GLdraw; $(MAKE)
	 cd optimization; $(MAKE)
	 cd geometry; $(MAKE)
	 cd meshing; $(MAKE)
	 cd image; $(MAKE)
	 cd statistics; $(MAKE)
	 cd robotics; $(MAKE)
	 cd planning; $(MAKE)
	 cd spline; $(MAKE)

KrisLibrary: all 
	mkdir $(LIBDIROUT)
	 $(AR) rcs $(LIBDIROUT)/libKrisLibrary.a $(addsuffix /$(OBJDIR)/*.o,$(SUB_LIBS)) $(KRISLIBRARY_EXTRAOBJS)
	 $(RANLIB) $(LIBDIROUT)/libKrisLibrary.a

docs:
	 doxygen doxygen.conf


include ../Makefile.config
SRCS= $(wildcard *.cpp)
INCDIR= ../ $(OPTIMIZATION_INCDIR)
DEFINES= $(OPTIMIZATION_DEFINES)
LIBNAME= optimization
ifeq ($(HAVE_GLPK),1)
  OPTIMIZATION_EXTRAOBJS = $(OBJDIR)/glpk/*.o
endif
EXTRAOBJS= $(OPTIMIZATION_EXTRAOBJS)
include ../Makefile.template

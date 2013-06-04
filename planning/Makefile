include ../Makefile.config
SRCS=$(wildcard *.cpp)
LIBNAME= planning
INCDIR= ../ $(TINYXML)
DEFINES= HAVE_TINYXML=$(HAVE_TINYXML) TIXML_USE_STL
include ../Makefile.template

docs:
	doxygen doxygen.conf
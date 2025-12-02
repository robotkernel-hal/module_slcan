
libdir = $(prefix)/lib
lib_LTLIBRARIES = libmodule_slcan.la

libmodule_slcan_la_SOURCES  = slcanmaster.cpp

libmodule_slcan_la_CXXFLAGS = -Wall -I$(builddir) -I$(srcdir) \
                             @ROBOTKERNEL_CFLAGS@ \
                             @SERVICE_PROVIDER_PROCESS_DATA_INSPECTION_CFLAGS@ \
                             @SERVICE_PROVIDER_MEMORY_INSPECTION_CFLAGS@ \
                             @SERVICE_PROVIDER_CANOPEN_PROTOCOL_CFLAGS@ \
							 @SERIALCAN_CFLAGS@

libmodule_slcan_la_LDFLAGS  = -Wall -release $(PACKAGE_VERSION) @SERIALCAN_LIBS@


# Makefile for NPI Server


#Compilation output
OBJS = out

#temporary output for lib generation
TEMP = temp

#library to use
LIBS_x86= -lpthread
# -lusb-1.0
LIBS_armBeagleBoard= -lpthread
# -lusb-1.0
LIBS_armBeagleBone= -lpthread
# -lusb-1.0

#include path
INCLUDES= -I application\
	-I ipclib/server \
	-I ipclib/common

#compilation tools
CC_armBeagleBoard = arm-none-linux-gnueabi-gcc
CC_aragoBeagleBone = arm-arago-linux-gnueabi-gcc
# arm-arago-gnueabi-gcc is for default Angstrom that comes with BeagleBone
CC_armBeagleBone = arm-linux-gnueabihf-gcc
# arm-linux-gnueabihf-gcc is for Ubuntu on BeagleBone Black
# get it here: eewiki.net/display/linuxonarm/BeagleBone+Black#BeagleBoneBlack-ARMCrossCompiler:GCC
CC_x86 = gcc

#predefine
#DEFINES = -DRNP_HOST -D__BIG_DEBUG__
#DEFINES = -DRNP_HOST -DSRDY_INTERRUPT -DNPI_UNIX
DEFINES = -DMRDY_EARLY_FIX -DRNP_HOST -DNPI_SPI=TRUE -DNPI_UART=TRUE -DNPI_I2C=TRUE -D__DEBUG_TIME__ -D__DEBUG_TIME__I2C -DSRDY_INTERRUPT
# -D__DEBUG_TIME__HID -D__DEBUG_MUTEX__ -DNPI_HID=TRUE 

#compilation Option
COMPILO_FLAGS_x86 = "-Wall  $(INCLUDES) $(DEFINES)" 
COMPILO_FLAGS_armBeagleBoard = "-Wall  $(INCLUDES) $(DEFINES) -mcpu=cortex-a8 -march=armv7-a"
COMPILO_FLAGS_armBeagleBone = "-Wall  $(INCLUDES) $(DEFINES) -mcpu=cortex-a8 -march=armv7-a"

#list of object file to compile for the server
SERVER_OBJS= \
	$(OBJS)/npi_lnx_ipc.o \
	$(OBJS)/npi_lnx_uart.o \
	$(OBJS)/npi_lnx_spi.o \
	$(OBJS)/npi_lnx_i2c.o \
	$(OBJS)/hal_gpio.o \
	$(OBJS)/hal_i2c.o \
	$(OBJS)/hal_spi.o


#by default, do not use the library.
#PROJ_OBJS=$(SERVER_OBJS)

.PHONY: all clean lib create_output arch-all-x86 arch-all-armBeagleBoard arch-all-armBeagleBone exec_all_x86 exec_all_armBeagleBoard exec_all_armBeagleBone arch-all-x86 clean_obj clean_obj2 

all: \
	create_output \
	clean_obj \
	arch-all-x86 \
	clean_obj2 \
	arch-all-armBeagleBoard \
	clean_obj3 \
	arch-all-armBeagleBone \

create_output:
	@echo "********************************************************" 
	@echo "Check existance of output folder" 
	-if test ! -d $(OBJS); then mkdir $(OBJS); fi

arch-all-x86: 
	@echo "********************************************************" 
	@echo "COMPILING SERVER FOR x86" 
	@$(MAKE) COMPILO=$(CC_x86) COMPILO_FLAGS=$(COMPILO_FLAGS_x86) exec_all_x86

arch-all-armBeagleBoard: 
	@echo "********************************************************" 
	@echo "COMPILING SERVER FOR ARM BEAGLE BOARD" 
	@$(MAKE) COMPILO=$(CC_armBeagleBoard) COMPILO_FLAGS=$(COMPILO_FLAGS_armBeagleBoard) exec_all_armBeagleBoard

arch-all-armBeagleBone: 
	@echo "********************************************************" 
	@echo "COMPILING SERVER FOR ARM BEAGLE BONE" 
	@$(MAKE) COMPILO=$(CC_armBeagleBone) COMPILO_FLAGS=$(COMPILO_FLAGS_armBeagleBone) exec_all_armBeagleBone
	@if [ -n "$(EXTRACT_FILES)" ]; then echo $(MAKEFILE_LIST) | $(EXTRACT_FILES) >> project_file_list.txt; fi
	@if [ -n "$(EXTRACT_FILES)" ]; then sort -u -V project_file_list.txt | grep -v /usr/ >actual_specific_project_file_list.txt; fi
	@if [ -n "$(EXTRACT_FILES)" ]; then sort -u -V project_file_list.txt | grep /usr/ >actual_standard_project_file_list.txt; fi

arch-all-aragoBeagleBone: 
	@echo "********************************************************" 
	@echo "COMPILING SERVER FOR ARM BEAGLE BONE" 
	@$(MAKE) COMPILO=$(CC_aragoBeagleBone) COMPILO_FLAGS=$(COMPILO_FLAGS_armBeagleBone) exec_all_armBeagleBone
	@if [ -n "$(EXTRACT_FILES)" ]; then echo $(MAKEFILE_LIST) | $(EXTRACT_FILES) >> project_file_list.txt; fi
	@if [ -n "$(EXTRACT_FILES)" ]; then sort -u -V project_file_list.txt | grep -v /usr/ >actual_specific_project_file_list.txt; fi
	@if [ -n "$(EXTRACT_FILES)" ]; then sort -u -V project_file_list.txt | grep /usr/ >actual_standard_project_file_list.txt; fi


exec_all_x86: $(OBJS)/NPI_lnx_x86_server

exec_all_armBeagleBoard: $(OBJS)/NPI_lnx_armBeagleBoard_server

exec_all_armBeagleBone: $(OBJS)/NPI_lnx_armBeagleBone_server

clean_obj:
	@echo "********************************************************" 
	@echo "Cleaning Existing object" 
	-if test -d $(OBJS); then rm -f $(OBJS)/*.o; fi

clean_obj2:
	@echo "********************************************************" 
	@echo "Cleaning Existing object" 
	-if test -d $(OBJS); then rm -f $(OBJS)/*.o; fi

clean_obj3:
	@echo "********************************************************" 
	@echo "Cleaning Existing object" 
	-if test -d $(OBJS); then rm -f $(OBJS)/*.o; fi

clean:
	@echo "********************************************************" 
	@echo "Cleaning all" 
	-if test -d $(OBJS); then rm -rf $(OBJS); fi
	rm -f project_file_list.txt



$(OBJS)/NPI_lnx_armBeagleBoard_server: $(SERVER_OBJS)
	@echo "Building target" $@ "..."
	@$(COMPILO) -o $@ $(SERVER_OBJS) $(LIBS_armBeagleBoard)
	@echo "********************************************************" 

$(OBJS)/NPI_lnx_armBeagleBone_server: $(SERVER_OBJS)
	@echo "Building target" $@ "..."
	@$(COMPILO) -o $@ $(SERVER_OBJS) $(LIBS_armBeagleBone)
	@echo "********************************************************" 

$(OBJS)/NPI_lnx_x86_server: $(SERVER_OBJS)
	@echo "Building target" $@ "..."
	@$(COMPILO) -o $@ $(SERVER_OBJS) $(LIBS_x86)
	@echo "********************************************************" 

$(OBJS)/npi_lnx_ipc.o: ipclib/server/npi_lnx_ipc.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi

$(OBJS)/npi_lnx_spi.o: ipclib/server/npi_lnx_spi.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi

$(OBJS)/npi_lnx_uart.o: ipclib/server/npi_lnx_uart.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi

$(OBJS)/npi_lnx_i2c.o: ipclib/server/npi_lnx_i2c.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi

#$(OBJS)/npi_lnx_hid.o: ipclib/server/npi_lnx_hid.c
#	@echo "Compiling" $< "..."
#	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<

$(OBJS)/hal_spi.o: ipclib/server/hal_spi.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi

$(OBJS)/hal_i2c.o: ipclib/server/hal_i2c.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi

#$(OBJS)/hid.o: ipclib/server/hid.c
#	@echo "Compiling" $< "..."
#	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<

$(OBJS)/hal_gpio.o: ipclib/server/hal_gpio.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi

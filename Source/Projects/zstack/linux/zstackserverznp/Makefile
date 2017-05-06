# Makefile for ZStack Linux Server w/ZNP

# Check to see if environment variables have been set
ifndef PROTOINC	
$(info Need to export the PROTOINC variable with the path to protobuf-c include directory)
ERROR_FLAG = 1
endif

ifndef PROTOLIB
$(info Need to export the PROTOLIB variable with the path to protobuf-c lib directory)
ERROR_FLAG = 1
endif

ifndef TCLIB
$(info Need to export the TCLIB variable with the path to the ti-sdk toolchain lib directory)
ERROR_FLAG = 1
endif

ifeq ($(ERROR_FLAG), 1)
$(error Stopping)
endif

# ARM Protobuf directories - Fill these in with the name of your protobuf-c directory
PROTOC_INC_DIR = $(PROTOINC)
PROTOC_LIB_PATH = -L$(PROTOLIB)

# Compilation tools - make sure to change the directory if its not called ti-sdk-am... 
ARM_LIB_PATH = -L$(TCLIB)
CC_arm = arm-linux-gnueabihf-gcc
CC_armArago = arm-arago-linux-gnueabi-gcc
CC_x86 = gcc
CC_arago = arm-arago-linux-gnueabi-gcc

# Variables for path locations
COMPONENTS = ../../../../Components
PROJECTS = ../../../../Projects

# Compilation outputl
OBJS = out

# Temporary output for lib generation
TEMP = temp

# Library to use
LIBS_x86= -lpthread -lprotobuf-c
LIBS_arm= -lpthread -lprotobuf-c

# Inlcude paths
INCLUDES = -I $(PROJECTS)/zstack/linux/zstackserverznp \
	-I $(PROJECTS)/zstack/linux/zstackpb\
	-I $(PROJECTS)/zstack/linux/srvwrapper \
	-I $(PROJECTS)/zstack/linux/linux_osal/include \
	-I $(PROJECTS)/zstack/linux/hal \
	-I $(PROJECTS)/zstack/linux/source \
	-I $(PROJECTS)/zstack/linux/zmac \
	-I $(COMPONENTS)/hal/include \
	-I $(COMPONENTS)/stack/nwk \
	-I $(COMPONENTS)/stack/zcl \
	-I $(COMPONENTS)/stack/sec \
	-I $(COMPONENTS)/stack/af \
	-I $(COMPONENTS)/stack/sys \
	-I $(COMPONENTS)/stack/zdo \
	-I $(COMPONENTS)/zmac \
	-I $(COMPONENTS)/mt \
	-I $(COMPONENTS)/mac/include

# Predefine
DEFINES = -DLINUX_ZNP -DZDO_COORDINATOR -D__ZNP_DEBUG__ -DLCD_SUPPORTED  \
	-DAPI_CLIENT_8BIT_LEN -DSERVER_NAME=ZSTACKZNP_SRVR \
	-D__APP_UI__

# Compilation Option
COMPILO_FLAGS_x86 = "-O0 -g3 -Wall $(INCLUDES) $(DEFINES) -include ../hal/hal_types.h -include f8wConfig.h" 
COMPILO_FLAGS_arm = "-O0 -g3 -Wall -Wno-unused-but-set-variable $(INCLUDES) $(DEFINES) -include ../hal/hal_types.h -include f8wConfig.h -mcpu=cortex-a8 -march=armv7-a -I $(PROTOC_INC_DIR)"

# List of object file to compile
APP_OBJS= \
	$(OBJS)/osal_zstack_server_znp.o \
	$(OBJS)/zmain.o \
	$(OBJS)/znp_af.o \
	$(OBJS)/znp_misc.o \
	$(OBJS)/znp_zdo.o \
	$(OBJS)/zstack.pb-c.o \
	$(OBJS)/zstackpb.o \
	$(OBJS)/api_client.o \
	$(OBJS)/trace.o \
	$(OBJS)/main.o \
	$(OBJS)/api_server.o \
	$(OBJS)/configparser.o \
	$(OBJS)/OSAL_Memory.o \
	$(OBJS)/OSAL_PwrMgr.o \
	$(OBJS)/OSAL_Timers.o \
	$(OBJS)/OSAL.o \
	$(OBJS)/saddr.o \
	$(OBJS)/OnBoard.o \
        $(OBJS)/hal_timer.o

.PHONY: all clean lib create_output arch-all-x86 arch-all-arm clean_obj clean_obj2

all: \
	create_output \
	clean_obj \
	arch-all-x86 \
	clean_obj2 \
	arch-all-arm \
	clean_obj3

create_output:
	@echo "********************************************************" 
	@echo "Check existance of output folder" 
	-if test ! -d $(OBJS); then mkdir $(OBJS); fi

arch-all-x86: 
	@echo "********************************************************" 
	@echo "COMPILING APP FOR x86" 
	@$(MAKE) COMPILO=$(CC_x86) COMPILO_FLAGS=$(COMPILO_FLAGS_x86) exec_app_x86

arch-all-arm: 
	@echo "********************************************************" 
	@echo "COMPILING APP FOR ARM" 
	@$(MAKE) COMPILO=$(CC_arm) COMPILO_FLAGS=$(COMPILO_FLAGS_arm) exec_app_arm

arch-all-arago: 
	@echo "********************************************************" 
	@echo "COMPILING APP FOR ARAGO" 
	@$(MAKE) COMPILO=$(CC_armArago) COMPILO_FLAGS=$(COMPILO_FLAGS_arm) exec_app_armArago

exec_app_x86: $(OBJS)/ZLSZNP_x86

exec_app_arm: $(OBJS)/ZLSZNP_arm

exec_app_armArago: $(OBJS)/ZLSZNP_arago

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

$(OBJS)/ZLSZNP_arm: $(APP_OBJS)
	@echo "Building target" $@ "..."
	@$(COMPILO) -rdynamic -o $@ $(APP_OBJS) $(ARM_LIB_PATH) $(PROTOC_LIB_PATH) $(LIBS_arm) 
	@if [ -n "$(EXTRACT_FILES)" ]; then echo $(MAKEFILE_LIST) | $(EXTRACT_FILES) >> project_file_list.txt; fi
	@if [ -n "$(EXTRACT_FILES)" ]; then sort -u -V project_file_list.txt | grep -v /usr/ >actual_specific_project_file_list.txt; fi
	@if [ -n "$(EXTRACT_FILES)" ]; then sort -u -V project_file_list.txt | grep /usr/ >actual_standard_project_file_list.txt; fi
	@echo "********************************************************" 

$(OBJS)/ZLSZNP_arago: $(APP_OBJS)
	@echo "Building target" $@ "..."
	@$(COMPILO) -rdynamic -o $@ $(APP_OBJS) $(ARM_LIB_PATH) $(PROTOC_LIB_PATH) $(LIBS_arm) 
	@if [ -n "$(EXTRACT_FILES)" ]; then echo $(MAKEFILE_LIST) | $(EXTRACT_FILES) >> project_file_list.txt; fi
	@if [ -n "$(EXTRACT_FILES)" ]; then sort -u -V project_file_list.txt | grep -v /usr/ >actual_specific_project_file_list.txt; fi
	@if [ -n "$(EXTRACT_FILES)" ]; then sort -u -V project_file_list.txt | grep /usr/ >actual_standard_project_file_list.txt; fi
	@echo "********************************************************" 

$(OBJS)/ZLSZNP_x86: $(APP_OBJS)
	@echo "Building target" $@ "..."
	@$(COMPILO) -rdynamic -o $@ $(APP_OBJS) $(LIBS_x86)
	@if [ -n "$(EXTRACT_FILES)" ]; then echo $(MAKEFILE_LIST) | $(EXTRACT_FILES) >> project_file_list.txt; fi
	@if [ -n "$(EXTRACT_FILES)" ]; then sort -u -V project_file_list.txt | grep -v /usr/ >actual_specific_project_file_list.txt; fi
	@if [ -n "$(EXTRACT_FILES)" ]; then sort -u -V project_file_list.txt | grep /usr/ >actual_standard_project_file_list.txt; fi
	@echo "********************************************************" 

# Start of Object file creation
$(OBJS)/osal_zstack_server_znp.o: osal_zstack_server_znp.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi

$(OBJS)/zmain.o: ../source/zmain.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi

$(OBJS)/znp_af.o: znp_af.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi

$(OBJS)/znp_misc.o: znp_misc.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi

$(OBJS)/znp_zdo.o: znp_zdo.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi

$(OBJS)/zstack.pb-c.o: ../zstackpb/zstack.pb-c.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi

$(OBJS)/zstackpb.o: ../zstackpb/zstackpb.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi

$(OBJS)/api_client.o: ../srvwrapper/api_client.c 
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi


$(OBJS)/trace.o: ../srvwrapper/trace.c 
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi

$(OBJS)/main.o: ../srvwrapper/main.c 
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi

$(OBJS)/api_server.o: ../srvwrapper/api_server.c 
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi

$(OBJS)/configparser.o: ../srvwrapper/configparser.c 
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi

$(OBJS)/OSAL_Memory.o: ../linux_osal/common/OSAL_Memory.c 
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi

$(OBJS)/OSAL_PwrMgr.o: ../linux_osal/common/OSAL_PwrMgr.c 
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi

$(OBJS)/OSAL_Timers.o: ../linux_osal/common/OSAL_Timers.c 
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi

$(OBJS)/OSAL.o: ../linux_osal/common/OSAL.c 
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi

$(OBJS)/saddr.o: $(COMPONENTS)/services/saddr/saddr.c 
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi

$(OBJS)/OnBoard.o: ../source/OnBoard.c 
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi

$(OBJS)/hal_timer.o: ../hal/hal_timer.c 
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi

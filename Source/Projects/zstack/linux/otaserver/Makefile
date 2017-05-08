# Makefile for Linux OTA Server Application

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
CC_arm = mipsel-openwrt-linux-gcc
#arm-linux-gnueabihf-gcc
CC_arago = arm-arago-linux-gnueabi-gcc
CC_x86 = gcc

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
INCLUDES = -I $(PROJECTS)/zstack/linux/otaserver \
	-I $(PROJECTS)/zstack/linux/zstackpb \
	-I $(PROJECTS)/zstack/linux/otaserver \
	-I $(PROJECTS)/zstack/linux/srvwrapper \
    	-I $(PROJECTS)/zstack/linux/serverpb \
	-I $(PROJECTS)/zstack/linux/nwkmgr \
	-I $(PROJECTS)/zstack/linux/sdb \
        -I $(PROJECTS)/zstack/linux/source \
	-I $(PROJECTS)/zstack/linux/linux_osal/include \
	-I $(PROJECTS)/zstack/linux/hal \
	-I $(PROJECTS)/zstack/OTA/Source \
	-I $(COMPONENTS)/stack/zcl \
        -I $(COMPONENTS)/hal/include 
#	-I $(PROJECTS)/zstack/linux/zmac

# Predefine
DEFINES = -DZCL_STANDALONE \
	-DZG_SECURE_ENABLED \
	-DMAX_BINDING_CLUSTER_IDS \
	-DOTA_SERVER \
	-DHAL_OTA_BOOT_CODE \
	-DZCL_READ \
	-DOTA_HA \
	-DSERVER_NAME=OTASRVR
#	-D__APP_UI__ \

# Compilation Option
COMPILO_FLAGS_x86 = " -g -Wall $(INCLUDES) $(DEFINES) -include hal_types.h -include AF.h -include zcl.h -include zcl_ota.h "
COMPILO_FLAGS_arm = " -Wall $(INCLUDES) $(DEFINES) -include hal_types.h -include AF.h -include zcl.h -include zcl_ota.h -march=24kec -I $(PROTOC_INC_DIR)"

# List of object file to compile
APP_OBJS= \
	$(OBJS)/zstack.pb-c.o \
	$(OBJS)/zcl_general.o \
	$(OBJS)/zcl.o \
	$(OBJS)/zcl_ota.o \
	$(OBJS)/api_client.o \
	$(OBJS)/api_server.o \
	$(OBJS)/main.o \
	$(OBJS)/zcl_otaserver_lnx.o \
	$(OBJS)/OtaServer.o \
	$(OBJS)/OtaServer_db.o \
	$(OBJS)/SimpleDB.o \
	$(OBJS)/SimpleDBTxt.o \
	$(OBJS)/configparser.o \
	$(OBJS)/ota_common.o \
	$(OBJS)/otasrvr.o \
	$(OBJS)/otasrvr.pb-c.o \
	$(OBJS)/server.pb-c.o \
	$(OBJS)/trace.o \
	$(OBJS)/gatewayp2p.o \
	$(OBJS)/zcl_port.o

.PHONY: all clean lib create_output arch-all-x86 arch-all-arm clean_obj clean_obj2

all: create_output clean_obj arch-all-x86 clean_obj2 arch-all-arm

create_output:
	@echo "********************************************************" 
	@echo "Check existance of output folder" 
	-if test ! -d $(OBJS); then mkdir $(OBJS); fi

arch-all-x86: 
	@echo "********************************************************" 
	@echo "COMPILING APP FOR x86" 
	export ARCH=x86
	@$(MAKE) COMPILO=$(CC_x86) COMPILO_FLAGS=$(COMPILO_FLAGS_x86) exec_app_x86

arch-all-arm: 
	@echo "********************************************************" 
	@echo "COMPILING APP FOR ARM" 
	export ARCH=arm
	@$(MAKE) COMPILO=$(CC_arm) COMPILO_FLAGS=$(COMPILO_FLAGS_arm) exec_app_arm

arch-all-arago:
	@echo "********************************************************" 
	@echo "COMPILING APP FOR ARM" 
	export ARCH=arm
	@$(MAKE) COMPILO=$(CC_arago) COMPILO_FLAGS=$(COMPILO_FLAGS_arm) exec_app_arago

exec_app_x86: $(OBJS)/OTA_SRVR_x86

exec_app_arm: $(OBJS)/OTA_SRVR_arm

exec_app_arago: $(OBJS)/OTA_SRVR_arago

clean_obj:
	@echo "********************************************************" 
	@echo "Cleaning Existing object" 
	-if test -d $(OBJS); then rm -f $(OBJS)/*.o; fi

clean_obj2:
	@echo "********************************************************" 
	@echo "Cleaning Existing object" 
	-if test -d $(OBJS); then rm -f $(OBJS)/*.o; fi

clean:
	@echo "********************************************************" 
	@echo "Cleaning all" 
	-if test -d $(OBJS); then rm -rf $(OBJS); fi

$(OBJS)/OTA_SRVR_arm: $(APP_OBJS)
	@echo "Building target" $@ "..."
	@$(COMPILO) -o $@ $(APP_OBJS) $(ARM_LIB_PATH) $(PROTOC_LIB_PATH) $(LIBS_arm) 
	@if [ -n "$(EXTRACT_FILES)" ]; then echo $(MAKEFILE_LIST) | $(EXTRACT_FILES) >> project_file_list.txt; fi
	@if [ -n "$(EXTRACT_FILES)" ]; then sort -u -V project_file_list.txt | grep -v /usr/ >actual_specific_project_file_list.txt; fi
	@if [ -n "$(EXTRACT_FILES)" ]; then sort -u -V project_file_list.txt | grep /usr/ >actual_standard_project_file_list.txt; fi
	@echo "********************************************************" 

$(OBJS)/OTA_SRVR_arago: $(APP_OBJS)
	@echo "Building target" $@ "..."
	@$(COMPILO) -o $@ $(APP_OBJS) $(ARM_LIB_PATH) $(PROTOC_LIB_PATH) $(LIBS_arm) 
	@if [ -n "$(EXTRACT_FILES)" ]; then echo $(MAKEFILE_LIST) | $(EXTRACT_FILES) >> project_file_list.txt; fi
	@if [ -n "$(EXTRACT_FILES)" ]; then sort -u -V project_file_list.txt | grep -v /usr/ >actual_specific_project_file_list.txt; fi
	@if [ -n "$(EXTRACT_FILES)" ]; then sort -u -V project_file_list.txt | grep /usr/ >actual_standard_project_file_list.txt; fi
	@echo "********************************************************" 


$(OBJS)/OTA_SRVR_x86: $(APP_OBJS)
	@echo "Building target" $@ "..."
	@$(COMPILO) -o $@ $(APP_OBJS) $(LIBS_x86)
	@if [ -n "$(EXTRACT_FILES)" ]; then echo $(MAKEFILE_LIST) | $(EXTRACT_FILES) >> project_file_list.txt; fi
	@if [ -n "$(EXTRACT_FILES)" ]; then sort -u -V project_file_list.txt | grep -v /usr/ >actual_specific_project_file_list.txt; fi
	@if [ -n "$(EXTRACT_FILES)" ]; then sort -u -V project_file_list.txt | grep /usr/ >actual_standard_project_file_list.txt; fi
	@echo "********************************************************" 

# Start of Object file creation
$(OBJS)/zstack.pb-c.o: ../zstackpb/zstack.pb-c.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi

$(OBJS)/zcl_general.o: $(COMPONENTS)/stack/zcl/zcl_general.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi

$(OBJS)/zcl.o: $(COMPONENTS)/stack/zcl/zcl.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi

$(OBJS)/zcl_ota.o: ./zcl_ota.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi

$(OBJS)/api_client.o: ../srvwrapper/api_client.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi

$(OBJS)/api_server.o: ../srvwrapper/api_server.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi

$(OBJS)/main.o: ../srvwrapper/main.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi

$(OBJS)/configparser.o: ../srvwrapper/configparser.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi

$(OBJS)/OSAL.o: ../linux_osal/common/OSAL.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi

$(OBJS)/OSAL_Memory.o: ../linux_osal/common/OSAL_Memory.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi

$(OBJS)/OtaServer_db.o: OtaServer_db.c 
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $< 
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi

$(OBJS)/OtaServer.o: OtaServer.c 
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $< 
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi

$(OBJS)/zcl_otaserver_lnx.o: zcl_otaserver_lnx.c 
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $< 
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi

$(OBJS)/zcl_port.o: zcl_port.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi

$(OBJS)/ota_common.o: ota_common.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi

$(OBJS)/otasrvr.o: otasrvr.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi

$(OBJS)/otasrvr.pb-c.o: otasrvr.pb-c.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi

$(OBJS)/server.pb-c.o: ../serverpb/server.pb-c.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<		
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi

$(OBJS)/gatewayp2p.o: ../serverpb/gatewayp2p.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi

$(OBJS)/trace.o: ../srvwrapper/trace.c 
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<
	
$(OBJS)/SimpleDB.o: ../sdb/SimpleDB.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi

$(OBJS)/SimpleDBTxt.o: ../sdb/SimpleDBTxt.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi

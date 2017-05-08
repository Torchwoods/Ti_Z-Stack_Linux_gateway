# Makefile for Gateway Server

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
CC_x86 = gcc
CC_arago = arm-arago-linux-gnueabi-gcc

# Variables for path locations
COMPONENTS = ../../../../Components
PROJECTS = ../../../../Projects

# Compilation output
OBJS = out

# Temporary output for lib generation
TEMP = temp

# Library to use
LIBS_x86= -lpthread -lprotobuf-c
LIBS_arm= -lpthread -lprotobuf-c

# Include paths
INCLUDES = -I ../zstackpb \
	-I ../hal \
	-I . \
	-I $(COMPONENTS)/stack/zcl \
	-I $(PROJECTS)/zstack/linux/hagateway \
	-I $(PROJECTS)/zstack/linux/nwkmgr \
	-I $(PROJECTS)/zstack/linux/srvwrapper \
	-I $(PROJECTS)/zstack/linux/serverpb \
	-I $(PROJECTS)/zstack/linux/srvepconfig \
	-I $(PROJECTS)/zstack/linux/linux_osal/include \
	-I $(PROJECTS)/zstack/linux/hal \
	-I $(PROJECTS)/zstack/linux/zmac 

# Predefine
DEFINES = -g \
	-DZG_SECURE_ENABLED \
	-DMAX_BINDING_CLUSTER_IDS \
	-DZCL_GROUPS \
	-DZCL_SCENES \
	-DZCL_ON_OFF \
	-DZCL_STANDALONE \
	-DZCL_READ \
	-DZCL_WRITE \
	-DZCL_DISCOVER \
	-DZCL_ALARMS \
	-DZCL_REPORT \
	-DZCL_IDENTIFY \
	-DZCL_LEVEL_CTRL \
	-DZCL_POLL_CONTROL \
	-DZCL_HVAC_CLUSTER \
	-DZCL_WINDOWCOVERING \
	-DZCL_DOORLOCK \
	-DZCL_ACE \
	-DZCL_ZONE \
	-DSERVER_NAME=GATEWAY \
	-D__APP_UI__

#	-D__APP_UI__ \

# Compilation Option
COMPILO_FLAGS_x86 = "-Wall $(INCLUDES) $(DEFINES) -include ../hal/hal_types.h" 
COMPILO_FLAGS_arm = "-Wall $(INCLUDES) $(DEFINES) -include ../hal/hal_types.h -march=24kec -I $(PROTOC_INC_DIR)"

# List of object file to compile
APP_OBJS= \
	$(OBJS)/serverep.o \
	$(OBJS)/serverdefep.o \
	$(OBJS)/gatewaysrvr.o \
	$(OBJS)/trace.o \
	$(OBJS)/gatewayservices.o \
	$(OBJS)/gatewayp2p.o \
	$(OBJS)/gateway.pb-c.o \
	$(OBJS)/zstack.pb-c.o \
	$(OBJS)/server.pb-c.o \
	$(OBJS)/api_client.o \
	$(OBJS)/main.o \
	$(OBJS)/api_server.o \
	$(OBJS)/configparser.o \
	$(OBJS)/zcl_general.o \
	$(OBJS)/zcl_poll_control.o \
	$(OBJS)/zcl_lighting.o \
	$(OBJS)/zcl_closures.o \
	$(OBJS)/zcl_hvac.o \
	$(OBJS)/zcl_ss.o \
	$(OBJS)/zcl.o \
	$(OBJS)/aps_groups.o \
	$(OBJS)/zcl_port.o 

.PHONY: all clean lib create_output arch-all-x86 arch-all-arm clean_obj clean_obj2

all: \
	create_output \
	clean_obj \
	arch-all-x86 \
	clean_obj2 \
	arch-all-arm \

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
	@echo "COMPILING APP FOR ARM ARAGO" 
	@$(MAKE) COMPILO=$(CC_arago) COMPILO_FLAGS=$(COMPILO_FLAGS_arm) exec_app_arm_arago

exec_app_x86: $(OBJS)/GATEWAY_SRVR_x86

exec_app_arm: $(OBJS)/GATEWAY_SRVR_arm

exec_app_arm_arago: $(OBJS)/GATEWAY_SRVR_arago

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

$(OBJS)/GATEWAY_SRVR_arm: $(APP_OBJS)
	@echo "Building target" $@ "..."
	@$(COMPILO) -rdynamic -o $@ $(APP_OBJS) $(ARM_LIB_PATH) $(PROTOC_LIB_PATH) $(LIBS_arm) 
	@if [ -n "$(EXTRACT_FILES)" ]; then echo $(MAKEFILE_LIST) | $(EXTRACT_FILES) >> project_file_list.txt; fi
	@if [ -n "$(EXTRACT_FILES)" ]; then sort -u -V project_file_list.txt | grep -v /usr/ >actual_specific_project_file_list.txt; fi
	@if [ -n "$(EXTRACT_FILES)" ]; then sort -u -V project_file_list.txt | grep /usr/ >actual_standard_project_file_list.txt; fi
	@echo "********************************************************" 

$(OBJS)/GATEWAY_SRVR_x86: $(APP_OBJS)
	@echo "Building target" $@ "..."
	@$(COMPILO) -rdynamic -o $@ $(APP_OBJS) $(LIBS_x86)
	@if [ -n "$(EXTRACT_FILES)" ]; then echo $(MAKEFILE_LIST) | $(EXTRACT_FILES) >> project_file_list.txt; fi
	@if [ -n "$(EXTRACT_FILES)" ]; then sort -u -V project_file_list.txt | grep -v /usr/ >actual_specific_project_file_list.txt; fi
	@if [ -n "$(EXTRACT_FILES)" ]; then sort -u -V project_file_list.txt | grep /usr/ >actual_standard_project_file_list.txt; fi
	@echo "********************************************************" 

$(OBJS)/GATEWAY_SRVR_arago: $(APP_OBJS)
	@echo "Building target" $@ "..."
	@$(COMPILO) -rdynamic -o $@ $(APP_OBJS) $(ARM_LIB_PATH) $(PROTOC_LIB_PATH) $(LIBS_arm) 
	@if [ -n "$(EXTRACT_FILES)" ]; then echo $(MAKEFILE_LIST) | $(EXTRACT_FILES) >> project_file_list.txt; fi
	@if [ -n "$(EXTRACT_FILES)" ]; then sort -u -V project_file_list.txt | grep -v /usr/ >actual_specific_project_file_list.txt; fi
	@if [ -n "$(EXTRACT_FILES)" ]; then sort -u -V project_file_list.txt | grep /usr/ >actual_standard_project_file_list.txt; fi
	@echo "********************************************************"
	
# Start of Object file creation
$(OBJS)/zstack.pb-c.o: ../zstackpb/zstack.pb-c.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi
	
$(OBJS)/server.pb-c.o: ../serverpb/server.pb-c.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<	
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi

$(OBJS)/api_client.o: ../srvwrapper/api_client.c 
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
	
$(OBJS)/serverep.o: ../srvepconfig/serverep.c 
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $< 		
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi

$(OBJS)/serverdefep.o: ../srvepconfig/serverdefep.c 
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $< 		
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi

$(OBJS)/gatewaysrvr.o: gatewaysrvr.c 
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $< 
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi

$(OBJS)/gateway.pb-c.o: gateway.pb-c.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi

$(OBJS)/zcl_port.o: zcl_port.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi
		
$(OBJS)/zcl_general.o: $(COMPONENTS)/stack/zcl/zcl_general.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi
	
$(OBJS)/zcl_poll_control.o: $(COMPONENTS)/stack/zcl/zcl_poll_control.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<	
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi
	
$(OBJS)/zcl_lighting.o: $(COMPONENTS)/stack/zcl/zcl_lighting.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi
	
$(OBJS)/zcl_closures.o: $(COMPONENTS)/stack/zcl/zcl_closures.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi
	
$(OBJS)/zcl_hvac.o: $(COMPONENTS)/stack/zcl/zcl_hvac.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi
	
$(OBJS)/zcl_ss.o: $(COMPONENTS)/stack/zcl/zcl_ss.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<					
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi
		
$(OBJS)/zcl.o: $(COMPONENTS)/stack/zcl/zcl.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<		
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi
	
	
$(OBJS)/trace.o: ../srvwrapper/trace.c 
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<
	
$(OBJS)/gatewayservices.o: gatewayservices.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<	
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi
	
$(OBJS)/gatewayp2p.o: ../serverpb/gatewayp2p.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<	
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi

$(OBJS)/aps_groups.o: aps_groups.c 
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $< 
	@if [ -n "$(EXTRACT_FILES)" ]; then $(COMPILO) -M $< $(COMPILO_FLAGS) | $(EXTRACT_FILES) >> project_file_list.txt; fi

PROJ_DIR = .

OBJ_DIR = $(PROJ_DIR)/out

LIBS = -L$(PROTOLIB)

INCLUDE = -I$(PROJ_DIR) -I../engines -I../framework -I$(PROTOINC) -I../../nwkmgr -I../../hagateway -I../../otaserver


#CC=gcc
CC=arm-linux-gnueabihf-gcc
#CC=arm-arago-linux-gnueabi-gcc

CFLAGS= -c -Wall -g -std=gnu99

VPATH = $(PROJ_DIR):$(PROJ_DIR)/../framework:$(PROJ_DIR)/../engines:$(PROJ_DIR)/../../nwkmgr:$(PROJ_DIR)/../../hagateway:$(PROJ_DIR)/../../otaserver

SRCS = main.c polling.c timer_utils.c socket_interface.c tcp_client.c user_interface.c data_structures.c network_info_engine.c device_list_engine.c sensor_engine.c state_reflector.c attribute_engine.c actions_engine.c commissioning_engine.c system_engine.c ota_engine.c nwkmgr.pb-c.c gateway.pb-c.c otasrvr.pb-c.c group_scene_engine.c macros.c 

OBJS = $(SRCS:%.c=%.o) 

PROJECT_FOLDER = demo/project/

.PHONY: all clean create_output

all: create_output main.bin 

create_output:
	@echo "VPATH is $(VPATH)"
	@echo "********************************************************" 
	@echo "Check existance of output folder" 
	-if test ! -d $(OBJ_DIR); then mkdir $(OBJ_DIR); fi

main.bin: $(OBJS)
	@echo "Building $@..."
	$(CC) -rdynamic -o $@ $(OBJS) $(LIBS) -lprotobuf-c 
	@if [ -n "$(EXTRACT_FILES)" ]; then echo $(MAKEFILE_LIST) | $(EXTRACT_FILES) >> project_file_list.txt; fi
	@if [ -n "$(EXTRACT_FILES)" ]; then sort -u -V project_file_list.txt | grep -v /usr/ >actual_specific_project_file_list.txt; fi
	@if [ -n "$(EXTRACT_FILES)" ]; then sort -u -V project_file_list.txt | grep /usr/ >actual_standard_project_file_list.txt; fi
	mv $(OBJS) $(OBJ_DIR)

%.o: %.c 
	@echo "Compiling $< into $@"
	$(CC) $(CFLAGS) $(INCLUDE) $(DEFS) $< -o $@
	@if [ -n "$(EXTRACT_FILES)" ]; then $(CC) -M $< $(INCLUDE) $(DEFS) | $(EXTRACT_FILES) >> project_file_list.txt; fi
# rule for cleaning files generated during compilations.
clean:
	@echo "Cleaning up $(OBJS) $(OBJ_DIR)/*"	
	/bin/rm -f main.bin $(OBJS) $(OBJ_DIR)/*
	rm -f project_file_list.txt

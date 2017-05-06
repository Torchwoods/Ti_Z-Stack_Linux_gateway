PROJ_DIR = .

OBJ_DIR = $(PROJ_DIR)/out

INCLUDE = -I$(PROJ_DIR)


#CC=gcc
CC=arm-linux-gnueabihf-gcc
#CC=arm-arago-linux-gnueabi-gcc

CFLAGS= -c -Wall -g -std=gnu99

VPATH = $(PROJ_DIR)

SRCS = ../source/bbb_usbreset.c
OBJS = $(SRCS:%.c=%.o) 

PROJECT_FOLDER = .

.PHONY: all clean create_output

all: create_output bbb_usbreset.bin 

create_output:
	@echo "VPATH is $(VPATH)"
	@echo "********************************************************" 
	@echo "Check existance of output folder" 
	-if test ! -d $(OBJ_DIR); then mkdir $(OBJ_DIR); fi

bbb_usbreset.bin: $(OBJS)
	@echo "Building $@..."
	$(CC) -o $@ $(OBJS) $(LIBS) 	
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
	/bin/rm -f bbb_usbreset.bin $(OBJS) $(OBJ_DIR)/*

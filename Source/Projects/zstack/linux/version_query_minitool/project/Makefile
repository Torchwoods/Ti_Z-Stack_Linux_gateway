
SBU_REV= "0.1"

CC=mipsel-openwrt-linux-gcc
#arm-arago-linux-gnueabi-gcc

CFLAGS = -Wall -DVERSION_NUMBER=${SBU_REV}
OBJECTS = main.o zbSocCmd.o zbSocTransportUart.o
LIBS = -L$(TCLIB)../usr/lib/ 
#-lrt -lcurses

DEFS += -D_GNU_SOURCE -DxHAL_UART_SPI

APP_NAME=gw_soc_fw_version_query.bin

.PHONY: all, clean

${APP_NAME}: ${OBJECTS}
	$(CC) $(CFLAGS) $(OBJECTS) $(LIBS) -o ${APP_NAME}
	@if [ -n "$(EXTRACT_FILES)" ]; then echo $(MAKEFILE_LIST) | $(EXTRACT_FILES) >> project_file_list.txt; fi
	@if [ -n "$(EXTRACT_FILES)" ]; then sort -u -V project_file_list.txt | grep -v /usr/ >actual_specific_project_file_list.txt; fi
	@if [ -n "$(EXTRACT_FILES)" ]; then sort -u -V project_file_list.txt | grep /usr/ >actual_standard_project_file_list.txt; fi

%.o: ../Source/%.c
	${CC} ${CFLAGS} ${INCLUDE} ${DEFS} -c -o $@ $<
	@if [ -n "$(EXTRACT_FILES)" ]; then $(CC) -M $< $(INCLUDE) $(DEFS) | $(EXTRACT_FILES) >> project_file_list.txt; fi

all: ${APP_NAME}

clean:
	rm -rf *.o ${APP_NAME}

#Please set the following variables:
#  Mandatory:
#    NPI_SOURCE = Directory where Remo TI installed to build NPI server
#    PROTOLIB = Directory where the protobuf library is installed
#    PROTOINC = Directory where the protobuf include files are installed (Note: shall not be terminated with a '/')
#  Optional:
#    GATEWAY = Point to svn/trunk/ (Note: must be terminated with a '/')
#    PKGS_OUTPUT_DIR = Local dir to save resulting packages

#Please comment/uncomment the following definitions as required to control this script's output
#Package type:
	export BUILD_FULL_SOURCE_PACKAGE="TRUE"
#Package structure
	#export BINARIES_AS_DIRECTORY="TRUE"
	export BINARIES_AS_TAR="TRUE"
	export COMPLETE_PACKAGE_AS_DIRECTORY="TRUE"
	#export COMPLETE_PACKAGE_AS_TAR="TRUE"
#Target platform:
	export TARGET_PLATFORM="BEAGLEBONE_BLACK"
	#export TARGET_PLATFORM="x86"

#Version
	export VERSION="101"
	export SVN_VERSION=`svnversion`

echo "SVN Version = " $SVN_VERSION

#The following flags are ONLY for debug the script, and shall be kept commented out
	#export SKIP_BUILDING="TRUE"


# *** Check arguments ***********************************************************************************

pushd `dirname $0` > /dev/null
SCRIPTPATH=`pwd`
popd > /dev/null


echo
echo "Main project\'s base directory: GATEWAY=$GATEWAY (leave blank to use the detault)"
echo NPI Project\'s base directory NPI_SOURCE=$NPI_SOURCE
echo Protobuf library directory: PROTOLIB=$PROTOLIB
echo Protobuf include directory: PROTOINC=$PROTOINC
echo "Project output subdirectory: PKGS_OUTPUT_DIR=$PKGS_OUTPUT_DIR (leave blank to use the detault)"
echo


if [ -n "$GATEWAY" ]; then
	echo dummy > /dev/null
else
    export GATEWAY=`readlink -m $SCRIPTPATH/../../../../`/
	echo "Main project\'s base directory defaulted to $GATEWAY"
fi


if [ -n "$PROTOLIB" ]; then
	echo dummy > /dev/null
else 
	echo "ERROR: Environment variable PROTOLIB not set. It should point to the protobuf library directory."
	export MISSING_ENV="TRUE"
fi


if [ -n "$PROTOINC" ]; then
	echo dummy > /dev/null
else 
	echo "ERROR: Environment variable PROTOINC not set. I t should point to the protobuf include directory."
	export MISSING_ENV="TRUE"
fi


if [ -n "$NPI_SOURCE" ] ; then 
    echo dummy > /dev/null
else
    export NPI_SOURCE=`readlink -m $SCRIPTPATH/../RemoTI-Linux-master/`/
	echo "NPI Source directory defaulted to $NPI_SOURCE"
fi


if [ -n "$MISSING_ENV" ]; then
	echo "ERROR: Missing environment variables (see above). Please define them and rerun the script. Aborting now."
	exit
fi

if [ $TARGET_PLATFORM == "BEAGLEBONE_BLACK" ]; then
	#export COMPILER=arm-linux-gnueabihf-gcc
	export COMPILER=mipsel-openwrt-linux-gcc
	export BUILD_TYPE=arch-all-arm
	export PLATFORM_SUBSTRING=arm
elif [ $TARGET_PLATFORM == "x86" ]; then
	export COMPILER=gcc
	export BUILD_TYPE=arch-all-x86
	export PLATFORM_SUBSTRING=x86
else
	echo --- $TARGET_PLATFORM
	echo "ERROR: Target platform not set correctly. Please set TARGET_PLATFORM to either BEAGLEBONE_BLACK or x86"
	exit
fi

export PACKAGE_TYPE=BINARY
if [ -n "$BUILDING_FROM_SOURCE_PACKAGE" ]; then
	export PACKAGE_TYPE=RUNTIME
else
	if [ -n "$BUILD_FULL_SOURCE_PACKAGE" ]; then
		export PACKAGE_TYPE=SOURCE
	fi
fi


if [ -n "$PKGS_OUTPUT_DIR" ]; then
	echo dummy > /dev/null
else
    export PKGS_OUTPUT_DIR=$SCRIPTPATH/ZStackGwRefDes_${VERSION}_${PACKAGE_TYPE}_$PLATFORM_SUBSTRING
	echo "Output directory defaulted to $PKGS_OUTPUT_DIR"
fi

echo Building $PACKAGE_TYPE package for $TARGET_PLATFORM platform


export DOCS_DIR=$PKGS_OUTPUT_DIR/Documents
export FIRMWARE_DIR=$PKGS_OUTPUT_DIR/Firmware
export SOURCE_DIR=$PKGS_OUTPUT_DIR/Source
export PROTO_FILES_DIR=$PKGS_OUTPUT_DIR/Proto_files

export BINARIES_DIR=$PKGS_OUTPUT_DIR/Precompiled_$PLATFORM_SUBSTRING
export BINARIES_APP_DIR=$BINARIES_DIR/app
export BINARIES_MISC_DIR=$BINARIES_DIR/misc
export BINARIES_PROTOBUF_LIB_DIR=$BINARIES_DIR/protobuf
export BINARIES_SERVERS_DIR=$BINARIES_DIR/servers
export BINARIES_OTA_SAMPLE_IMAGES_DIR=$BINARIES_MISC_DIR/ota_sample_images
export BINARIES_TOOLS_DIR=$BINARIES_DIR/tools

export SOURCE_PACKAGE_NAME=$SOURCE_DIR/gateway_source_package
export SAMPLE_APP_SOURCE_PACKAGE=$SOURCE_DIR/sample_app_source

export MAKE_LOG_FILE=$PKGS_OUTPUT_DIR/temp_make_log_file.txt
echo


# *** Create directories ***********************************************************************************


if [ -d "$PKGS_OUTPUT_DIR" ]; then
	rm -rf $PKGS_OUTPUT_DIR
	if [ -d "$PKGS_OUTPUT_DIR" ]; then
		echo "ERROR: Cannot clean $PKGS_OUTPUT_DIR - Aborting now."
		exit
	fi
fi

if [ -a "$PKGS_OUTPUT_DIR.tar" ]; then
	rm $PKGS_OUTPUT_DIR.tar
	if [ -a "$PKGS_OUTPUT_DIR.tar" ]; then
		echo "ERROR: Cannot clean $PKGS_OUTPUT_DIR.tar - Aborting now."
		exit
	fi
fi

export ERRORED_PKGS_OUTPUT_DIR=$(dirname $PKGS_OUTPUT_DIR)/ERROR__$(basename $PKGS_OUTPUT_DIR)

if [ -d "$ERRORED_PKGS_OUTPUT_DIR" ]; then
	rm -rf $ERRORED_PKGS_OUTPUT_DIR
	if [ -d "$ERRORED_PKGS_OUTPUT_DIR" ]; then
		echo "ERROR: Cannot clean $ERRORED_PKGS_OUTPUT_DIR - Aborting now."
		exit
	fi
fi

if [ -a "$ERRORED_PKGS_OUTPUT_DIR.tar" ]; then
	rm $ERRORED_PKGS_OUTPUT_DIR.tar
	if [ -a "$ERRORED_PKGS_OUTPUT_DIR.tar" ]; then
		echo "ERROR: Cannot clean $ERRORED_PKGS_OUTPUT_DIR.tar - Aborting now."
		exit
	fi
fi


if [ -n "$BUILDING_FROM_SOURCE_PACKAGE" ]; then
	echo dummy > /dev/null
else
	mkdir -p $DOCS_DIR
	mkdir -p $FIRMWARE_DIR
	mkdir -p $SOURCE_DIR
	mkdir -p $PROTO_FILES_DIR

	export EXTRACT_FILES_SCRIPT_NAME=extract_files
	export EXTRACT_FILES=$SCRIPTPATH/$EXTRACT_FILES_SCRIPT_NAME
fi


mkdir -p $BINARIES_DIR
mkdir -p $BINARIES_APP_DIR
mkdir -p $BINARIES_MISC_DIR
mkdir -p $BINARIES_PROTOBUF_LIB_DIR
mkdir -p $BINARIES_SERVERS_DIR
mkdir -p $BINARIES_OTA_SAMPLE_IMAGES_DIR
mkdir -p $BINARIES_TOOLS_DIR

# *** Build binaries and copy configuration files ***********************************************************************************

if [ -n "$SKIP_BUILDING" ]; then
	echo dummy > /dev/null
else

	cd $NPI_SOURCE/Projects/tools/LinuxHost
	make clean
	make create_output
	make arch-all-armBeagleBone CC_armBeagleBone=$COMPILER |& tee -a $MAKE_LOG_FILE

	cd $GATEWAY/Projects/zstack/linux/zstackserverznp/
	make clean
	make create_output
	make $BUILD_TYPE |& tee -a $MAKE_LOG_FILE
	 
	cd $GATEWAY/Projects/zstack/linux/nwkmgr/
	make clean
	make create_output
	make $BUILD_TYPE |& tee -a $MAKE_LOG_FILE
	 
	cd $GATEWAY/Projects/zstack/linux/hagateway/
	make clean
	make create_output
	make $BUILD_TYPE |& tee -a $MAKE_LOG_FILE

	cd $GATEWAY/Projects/zstack/linux/otaserver/
	make clean
	make create_output
	make $BUILD_TYPE |& tee -a $MAKE_LOG_FILE

	cd $GATEWAY/Projects/zstack/linux/demo/project
	make clean
	make CC=$COMPILER |& tee -a $MAKE_LOG_FILE

	cd $GATEWAY/Projects/zstack/linux/sbl_tool/project
	make clean
	make CC=$COMPILER |& tee -a $MAKE_LOG_FILE

	cd $GATEWAY/Projects/zstack/linux/version_query_minitool/project
	make clean
	make CC=$COMPILER |& tee -a $MAKE_LOG_FILE

	cd $GATEWAY/Projects/zstack/linux/bbb_usbreset/project
	make clean
	make CC=$COMPILER |& tee -a $MAKE_LOG_FILE

fi

# *** Copy resources ***********************************************************************************

cp $NPI_SOURCE/Projects/tools/LinuxHost/out/NPI_lnx_armBeagleBone_server $BINARIES_SERVERS_DIR/NPI_lnx_${PLATFORM_SUBSTRING}_server
cp $GATEWAY/Projects/zstack/linux/collateral/gateway/config/NPI_Gateway.cfg $BINARIES_SERVERS_DIR/

cp $GATEWAY/Projects/zstack/linux/zstackserverznp/out/ZLSZNP_$PLATFORM_SUBSTRING $BINARIES_SERVERS_DIR/ZLSZNP_$PLATFORM_SUBSTRING
cp $GATEWAY/Projects/zstack/linux/zstackserverznp/config.ini $BINARIES_SERVERS_DIR/

cp $GATEWAY/Projects/zstack/linux/nwkmgr/out/NWKMGR_SRVR_$PLATFORM_SUBSTRING $BINARIES_SERVERS_DIR/

cp $GATEWAY/Projects/zstack/linux/hagateway/out/GATEWAY_SRVR_$PLATFORM_SUBSTRING $BINARIES_SERVERS_DIR/
cp $GATEWAY/Projects/zstack/linux/hagateway/gateway_config.tlg $BINARIES_SERVERS_DIR/

cp $GATEWAY/Projects/zstack/linux/otaserver/out/OTA_SRVR_$PLATFORM_SUBSTRING $BINARIES_SERVERS_DIR/

cp $GATEWAY/Projects/zstack/linux/demo/project/main.bin $BINARIES_APP_DIR/
cp $GATEWAY/Projects/zstack/linux/demo/project/sample_app_ota.cfg $BINARIES_SERVERS_DIR/
cp -r $GATEWAY/Projects/zstack/linux/otaserver/ota_sample_images/*.zigbee $BINARIES_OTA_SAMPLE_IMAGES_DIR/

cp $GATEWAY/Projects/zstack/linux/sbl_tool/project/sbl_tool.bin $BINARIES_TOOLS_DIR/
cp $GATEWAY/Projects/zstack/linux/version_query_minitool/project/gw_soc_fw_version_query.bin $BINARIES_TOOLS_DIR/

if [ $TARGET_PLATFORM == "BEAGLEBONE_BLACK" ]; then
	cp $GATEWAY/Projects/zstack/linux/bbb_usbreset/project/bbb_usbreset.bin $BINARIES_TOOLS_DIR/bbb_usbreset.bin
elif [ $TARGET_PLATFORM == "x86" ]; then
	echo dummy > /dev/null
fi

echo "BUILDING zigbeeHAgw script================================================"
cp $GATEWAY/Projects/zstack/linux/scripts/hagateway/track_servers $BINARIES_SERVERS_DIR/
cp $GATEWAY/Projects/zstack/linux/scripts/hagateway/zigbeeHAgw $BINARIES_SERVERS_DIR/
cp $GATEWAY/Projects/zstack/linux/scripts/sample_app/* $BINARIES_SERVERS_DIR/

cp "$PROTOLIB"/libprotobuf-c.so $BINARIES_PROTOBUF_LIB_DIR

cp $GATEWAY/Projects/zstack/linux/soc_images/CC2531_USB_Dongle/CC2531*.bin $BINARIES_MISC_DIR

if [ -n "$BUILDING_FROM_SOURCE_PACKAGE" ]; then
	echo dummy > /dev/null
else
	cp $GATEWAY/Projects/zstack/linux/collateral/gateway/documents/Common/*.pdf $DOCS_DIR/
	cp $GATEWAY/Projects/zstack/linux/collateral/gateway/documents/Common/*.txt $DOCS_DIR/
	cp $GATEWAY/Projects/zstack/linux/hagateway/*.proto $PROTO_FILES_DIR/
	cp $GATEWAY/Projects/zstack/linux/nwkmgr/*.proto $PROTO_FILES_DIR/
	cp $GATEWAY/Projects/zstack/linux/otaserver/*.proto $PROTO_FILES_DIR/

	cp $GATEWAY/Projects/zstack/linux/collateral/gateway/documents/BeagleBoneBlack/*.pdf $DOCS_DIR/
	cp $GATEWAY/Projects/zstack/linux/soc_images/CC2531_USB_Dongle/CC2531*.hex $FIRMWARE_DIR/
        cp $GATEWAY/Projects/zstack/linux/otaserver/ota_sample_images/SampleSwitchRouter_OTA.hex $FIRMWARE_DIR/
fi


# *** Create sample app source package ***********************************************************************************

if [ -n "$BUILDING_FROM_SOURCE_PACKAGE" ]; then
	echo dummy > /dev/null
else
	if [ -n "$BUILD_FULL_SOURCE_PACKAGE" ]; then
		echo dummy > /dev/null
	else
		cat $GATEWAY/Projects/zstack/linux/demo/project/actual_specific_project_file_list.txt >> $PKGS_OUTPUT_DIR/actual_specific_project_file_list_all.txt

		sort -u -V $PKGS_OUTPUT_DIR/actual_specific_project_file_list_all.txt > $PKGS_OUTPUT_DIR/sorted_actual_specific_project_file_list_all.txt
		rm $PKGS_OUTPUT_DIR/actual_specific_project_file_list_all.txt

		export LOCATION=$GATEWAY
		cd $LOCATION
		ESCAPED_LOCATION=$(echo $LOCATION | sed -e s?/?\\\\\/?g)
		cat $PKGS_OUTPUT_DIR/sorted_actual_specific_project_file_list_all.txt  | grep $ESCAPED_LOCATION | sed -e s?$LOCATION?? | tar -rvf $SAMPLE_APP_SOURCE_PACKAGE.tar -T - > /dev/null

		export LOCATION=$NPI_SOURCE
		cd $LOCATION
		ESCAPED_LOCATION=$(echo $LOCATION | sed -e s?/?\\\\\/?g)
		cat $PKGS_OUTPUT_DIR/sorted_actual_specific_project_file_list_all.txt  | grep $ESCAPED_LOCATION | sed -e s?$LOCATION?? | tar -rvf $SAMPLE_APP_SOURCE_PACKAGE.tar -T - > /dev/null

		export LOCATION=$PROTOINC/
		cd $LOCATION
		ESCAPED_LOCATION=$(echo $LOCATION | sed -e s?/?\\\\\/?g)
		cat $PKGS_OUTPUT_DIR/sorted_actual_specific_project_file_list_all.txt  | grep $ESCAPED_LOCATION | sed -e s?$LOCATION?? | tar -rvf $SAMPLE_APP_SOURCE_PACKAGE.tar -T - > /dev/null

		rm $PKGS_OUTPUT_DIR/sorted_actual_specific_project_file_list_all.txt

		cd $PROTOLIB/../../
		tar -hrvf $SAMPLE_APP_SOURCE_PACKAGE.tar protobuf-c-mips/lib/* > /dev/null

		echo 'pushd `dirname $0` > /dev/null' >>$PKGS_OUTPUT_DIR/build_sample_app
		echo 'SCRIPTPATH=`pwd`' >>$PKGS_OUTPUT_DIR/build_sample_app
		echo 'popd > /dev/null' >>$PKGS_OUTPUT_DIR/build_sample_app
		echo 'export PROTOINC=$SCRIPTPATH' >>$PKGS_OUTPUT_DIR/build_sample_app
		echo 'export PROTOLIB=$SCRIPTPATH'/protobuf-c-mips/lib >>$PKGS_OUTPUT_DIR/build_sample_app
		echo cd Projects/zstack/linux/demo/project >>$PKGS_OUTPUT_DIR/build_sample_app
		echo make clean >>$PKGS_OUTPUT_DIR/build_sample_app
		echo make CC=$COMPILER >>$PKGS_OUTPUT_DIR/build_sample_app
		echo 'rm -rf $SCRIPTPATH/out' >>$PKGS_OUTPUT_DIR/build_sample_app
		echo 'mkdir $SCRIPTPATH/out' >>$PKGS_OUTPUT_DIR/build_sample_app
		echo 'cp main.bin $SCRIPTPATH/out/' >>$PKGS_OUTPUT_DIR/build_sample_app

		cd $PKGS_OUTPUT_DIR
		chmod 777 build_sample_app
		tar -rvf $SAMPLE_APP_SOURCE_PACKAGE.tar build_sample_app > /dev/null
		rm $PKGS_OUTPUT_DIR/build_sample_app

	#	gzip $SAMPLE_APP_SOURCE_PACKAGE.tar
		tar -xvf $SAMPLE_APP_SOURCE_PACKAGE.tar -C $SOURCE_DIR/ > /dev/null
		rm $SAMPLE_APP_SOURCE_PACKAGE.tar

	fi
fi	


# *** Create source package ***********************************************************************************


if [ -n "$BUILDING_FROM_SOURCE_PACKAGE" ]; then
	echo dummy > /dev/null
else
	if [ -n "$BUILD_FULL_SOURCE_PACKAGE" ]; then

		cat $NPI_SOURCE/Projects/tools/LinuxHost/actual_specific_project_file_list.txt >> $PKGS_OUTPUT_DIR/actual_specific_project_file_list_all.txt
		cat $GATEWAY/Projects/zstack/linux/demo/project/actual_specific_project_file_list.txt >> $PKGS_OUTPUT_DIR/actual_specific_project_file_list_all.txt
		cat $GATEWAY/Projects/zstack/linux/zstackserverznp/actual_specific_project_file_list.txt >> $PKGS_OUTPUT_DIR/actual_specific_project_file_list_all.txt
		cat $GATEWAY/Projects/zstack/linux/nwkmgr/actual_specific_project_file_list.txt >> $PKGS_OUTPUT_DIR/actual_specific_project_file_list_all.txt
		cat $GATEWAY/Projects/zstack/linux/hagateway/actual_specific_project_file_list.txt >> $PKGS_OUTPUT_DIR/actual_specific_project_file_list_all.txt
		cat $GATEWAY/Projects/zstack/linux/otaserver/actual_specific_project_file_list.txt >> $PKGS_OUTPUT_DIR/actual_specific_project_file_list_all.txt
		cat $GATEWAY/Projects/zstack/linux/sbl_tool/project/actual_specific_project_file_list.txt >> $PKGS_OUTPUT_DIR/actual_specific_project_file_list_all.txt
		cat $GATEWAY/Projects/zstack/linux/version_query_minitool/project/actual_specific_project_file_list.txt >> $PKGS_OUTPUT_DIR/actual_specific_project_file_list_all.txt
		cat $GATEWAY/Projects/zstack/linux/bbb_usbreset/project/actual_specific_project_file_list.txt >> $PKGS_OUTPUT_DIR/actual_specific_project_file_list_all.txt

		echo $NPI_SOURCE/Projects/tools/LinuxHost/NPI_Gateway.cfg | $EXTRACT_FILES  >> $PKGS_OUTPUT_DIR/actual_specific_project_file_list_all.txt
		echo $GATEWAY/Projects/zstack/linux/zstackserverznp/config.ini | $EXTRACT_FILES  >> $PKGS_OUTPUT_DIR/actual_specific_project_file_list_all.txt
		echo $GATEWAY/Projects/zstack/linux/nwkmgr/nwkmgr_config.ini | $EXTRACT_FILES  >> $PKGS_OUTPUT_DIR/actual_specific_project_file_list_all.txt
		echo $GATEWAY/Projects/zstack/linux/hagateway/gateway_config.ini | $EXTRACT_FILES  >> $PKGS_OUTPUT_DIR/actual_specific_project_file_list_all.txt
		echo $GATEWAY/Projects/zstack/linux/hagateway/gateway_config.tlg | $EXTRACT_FILES  >> $PKGS_OUTPUT_DIR/actual_specific_project_file_list_all.txt
		echo $GATEWAY/Projects/zstack/linux/otaserver/otaserver.ini | $EXTRACT_FILES  >> $PKGS_OUTPUT_DIR/actual_specific_project_file_list_all.txt
		echo $GATEWAY/Projects/zstack/linux/demo/project/sample_app_ota.cfg | $EXTRACT_FILES  >> $PKGS_OUTPUT_DIR/actual_specific_project_file_list_all.txt
		echo $GATEWAY/Projects/zstack/linux/otaserver/ota_sample_images/*.zigbee | $EXTRACT_FILES  >> $PKGS_OUTPUT_DIR/actual_specific_project_file_list_all.txt
		echo $GATEWAY/Projects/zstack/linux/otaserver/ota_sample_images/Readme.txt | $EXTRACT_FILES  >> $PKGS_OUTPUT_DIR/actual_specific_project_file_list_all.txt
		echo $GATEWAY/Projects/zstack/linux/collateral/gateway/config/NPI_Gateway.cfg | $EXTRACT_FILES  >> $PKGS_OUTPUT_DIR/actual_specific_project_file_list_all.txt

		sort -u -V $PKGS_OUTPUT_DIR/actual_specific_project_file_list_all.txt > $PKGS_OUTPUT_DIR/sorted_actual_specific_project_file_list_all.txt
		rm $PKGS_OUTPUT_DIR/actual_specific_project_file_list_all.txt

		export LOCATION=$GATEWAY
		cd $LOCATION
		ESCAPED_LOCATION=$(echo $LOCATION | sed -e s?/?\\\\\/?g)
		cat $PKGS_OUTPUT_DIR/sorted_actual_specific_project_file_list_all.txt  | grep $ESCAPED_LOCATION | sed -e s?$LOCATION?? | tar -rvf $SOURCE_PACKAGE_NAME.tar -T - > /dev/null

		export LOCATION=$NPI_SOURCE
		cd $LOCATION
		ESCAPED_LOCATION=$(echo $LOCATION | sed -e s?/?\\\\\/?g)
		cat $PKGS_OUTPUT_DIR/sorted_actual_specific_project_file_list_all.txt  | grep $ESCAPED_LOCATION | sed -e s?$LOCATION?? | tar -rvf $SOURCE_PACKAGE_NAME.tar -T - > /dev/null

		export LOCATION=$PROTOINC/
		cd $LOCATION
		ESCAPED_LOCATION=$(echo $LOCATION | sed -e s?/?\\\\\/?g)
		cat $PKGS_OUTPUT_DIR/sorted_actual_specific_project_file_list_all.txt  | grep $ESCAPED_LOCATION | sed -e s?$LOCATION?? | tar -rvf $SOURCE_PACKAGE_NAME.tar -T - > /dev/null

		rm $PKGS_OUTPUT_DIR/sorted_actual_specific_project_file_list_all.txt
		
		cd $GATEWAY/Projects/zstack/linux/
		tar -rvf $SOURCE_PACKAGE_NAME.tar scripts/package_builder_bbb > /dev/null
		tar --transform=flags="r;s|scripts/hagateway|Projects/zstack/linux/scripts/hagateway|" -rvf $SOURCE_PACKAGE_NAME.tar --no-recursion scripts/hagateway/* > /dev/null
		tar --transform=flags="r;s|scripts/sample_app|Projects/zstack/linux/scripts/sample_app|" -rvf $SOURCE_PACKAGE_NAME.tar scripts/sample_app/* > /dev/null

		cd $PKGS_OUTPUT_DIR
		echo "Content list" > package_builder_expected_content_RUNTIME_$PLATFORM_SUBSTRING.txt
		echo "." > package_builder_expected_content_RUNTIME_$PLATFORM_SUBSTRING.txt
		echo "./content.txt" >> package_builder_expected_content_RUNTIME_$PLATFORM_SUBSTRING.txt
		find ./$(basename $BINARIES_DIR) | sort >> package_builder_expected_content_RUNTIME_$PLATFORM_SUBSTRING.txt
		tar --transform=flags="r;s|^|Projects/zstack/linux/scripts/|" -rvf $SOURCE_PACKAGE_NAME.tar package_builder_expected_content_RUNTIME_$PLATFORM_SUBSTRING.txt > /dev/null
		rm package_builder_expected_content_RUNTIME_$PLATFORM_SUBSTRING.txt
		
		cd $GATEWAY
		tar -rvf $SOURCE_PACKAGE_NAME.tar Projects/zstack/linux/demo/project/images/* > /dev/null

		tar -rvf $SOURCE_PACKAGE_NAME.tar Projects/zstack/linux/soc_images/CC2531_USB_Dongle/CC2531*.bin > /dev/null
		tar -rvf $SOURCE_PACKAGE_NAME.tar Projects/zstack/linux/soc_images/CC2531_USB_Dongle/CC2531*.hex > /dev/null

		cd $PROTOLIB/../../
		tar -hrvf $SOURCE_PACKAGE_NAME.tar protobuf-c-mips/lib/* > /dev/null
		
		echo 'pushd `dirname $0` > /dev/null' >>$PKGS_OUTPUT_DIR/build_all
		echo 'SCRIPTPATH=`pwd`' >>$PKGS_OUTPUT_DIR/build_all
		echo 'popd > /dev/null' >>$PKGS_OUTPUT_DIR/build_all
		echo 'export GATEWAY=$SCRIPTPATH' >>$PKGS_OUTPUT_DIR/build_all
		echo 'export NPI_SOURCE=$SCRIPTPATH/' >>$PKGS_OUTPUT_DIR/build_all
		echo 'export PROTOINC=$SCRIPTPATH' >>$PKGS_OUTPUT_DIR/build_all
		echo 'export PROTOLIB=$SCRIPTPATH'/protobuf-c-mips/lib >>$PKGS_OUTPUT_DIR/build_all
		echo 'export PKGS_OUTPUT_DIR=$SCRIPTPATH'/out >>$PKGS_OUTPUT_DIR/build_all
		echo 'cd scripts' >>$PKGS_OUTPUT_DIR/build_all
		echo 'export BUILDING_FROM_SOURCE_PACKAGE="TRUE"' >>$PKGS_OUTPUT_DIR/build_all
		echo ./package_builder_bbb >>$PKGS_OUTPUT_DIR/build_all

		cd $PKGS_OUTPUT_DIR
		chmod 777 build_all
		tar -rvf $SOURCE_PACKAGE_NAME.tar build_all > /dev/null
		rm $PKGS_OUTPUT_DIR/build_all
		
	#	gzip $SOURCE_PACKAGE_NAME.tar
		tar -xvf $SOURCE_PACKAGE_NAME.tar -C $SOURCE_DIR/  > /dev/null
		rm $SOURCE_PACKAGE_NAME.tar

	fi
fi

# *** Check package content ***********************************************************************************


# *** Tar executables ***********************************************************************************

echo
echo Package verification results:

export errors_count=$(grep -i -c "error" $MAKE_LOG_FILE)
export warnings_count=$(grep -i -c "warning" $MAKE_LOG_FILE)
rm $MAKE_LOG_FILE

if (($errors_count > 0))
then
    echo "ERROR: $errors_count error(s) occured during make"
	export PACKAGE_ERROR="TRUE"
fi

if (($warnings_count > 0))
then
    echo "ERROR: $warnings_count warning(s) occured during make"
#	export PACKAGE_ERROR="TRUE"
fi

cd $PKGS_OUTPUT_DIR
echo "Content list" > content.txt
find | sort > $PKGS_OUTPUT_DIR/content.txt

echo "diff $PKGS_OUTPUT_DIR/content.txt $GATEWAY/Projects/zstack/linux/scripts/package_builder_expected_content_${PACKAGE_TYPE}_$PLATFORM_SUBSTRING.txt"

diff $PKGS_OUTPUT_DIR/content.txt $GATEWAY/Projects/zstack/linux/scripts/package_builder_expected_content_${PACKAGE_TYPE}_$PLATFORM_SUBSTRING.txt
if [ $? -eq 0 ]
then
    echo "Content verified successfully" > /dev/null
else
    echo "ERROR: Content verification failed"
	export PACKAGE_ERROR="TRUE"
fi

if [ -n "$BINARIES_AS_TAR" ]; then
	cd $BINARIES_DIR
	tar -rvf $BINARIES_DIR.tar * > /dev/null
	if [ $? -eq 0 ]
	then
		echo "binaries tar successfull" > /dev/null
	else
		echo "ERROR: binaries tar failed"
		export PACKAGE_ERROR="TRUE"
	fi
fi

if [ -n "$BINARIES_AS_DIRECTORY" ]; then
	echo dummy > /dev/null
else
	rm -rf $BINARIES_DIR/*
fi

if [ -n "$BINARIES_AS_TAR" ]; then
	mv $BINARIES_DIR.tar $BINARIES_DIR/z-stack_linux_gateway_${PLATFORM_SUBSTRING}_binaries_$SVN_VERSION.tar
fi

if [ -n "$BUILDING_FROM_SOURCE_PACKAGE" ]; then
	echo dummy > /dev/null
else
	if [ -n "$COMPLETE_PACKAGE_AS_TAR" ]; then
		cd $PKGS_OUTPUT_DIR
		tar -rvf $PKGS_OUTPUT_DIR.tar * > /dev/null

		if [ $? -eq 0 ]
		then
			echo "package tar successfull" > /dev/null
		else
			echo "ERROR: package tar failed"
			export PACKAGE_ERROR="TRUE"
		fi
	fi

	if [ -n "$COMPLETE_PACKAGE_AS_DIRECTORY" ]; then
		echo dummy > /dev/null
	else
		rm -rf $PKGS_OUTPUT_DIR
	fi
fi

echo
if [ -n "$PACKAGE_ERROR" ]; then
	mv $PKGS_OUTPUT_DIR.tar $(dirname $PKGS_OUTPUT_DIR)/ERROR__$(basename $PKGS_OUTPUT_DIR).tar &> /dev/null
	mv $PKGS_OUTPUT_DIR $(dirname $PKGS_OUTPUT_DIR)/ERROR__$(basename $PKGS_OUTPUT_DIR) &> /dev/null
	echo "ERROR: $PACKAGE_TYPE package for $PLATFORM_SUBSTRING creation failed. See details above. Output folder/tar name prefixed with \"ERROR_.\""
else
	echo "OK: $PACKAGE_TYPE package for $PLATFORM_SUBSTRING created Successfully"
fi
echo

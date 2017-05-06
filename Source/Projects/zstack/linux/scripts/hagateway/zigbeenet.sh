#! /bin/sh
#
# zigbeenet.sh
#
# this should live in the /etc/init.d directory and be linked to by
# SXXzigbeenet to start and KXXzigbeenet to stop

ZIGBEEGW=/opt/TexasInstruments/ZigBeeHAGW

cd $ZIGBEEGW

trap status_running 	SIGUSR1
trap status_started 	SIGCONT
trap status_stopped  	SIGUSR2

echo $0 my PID is $$

status_running()
{
	echo "got a SIGUSR1 (running)!"
	gw_status="running"
}

status_started()
{
	echo "got a SIGCONT (started)!"
	gw_status="started"
}

status_stopped()
{
	echo "got a SIGUSR2 (stopped)!"
	gw_status="stopped"
}

case "$1" in
  start)
  	#./zigbeeHAgw >& /tmp/zigbeeHAgw.out &
  	./zigbeeHAgw &
	;;

  stop)
    killall -TERM zigbeeHAgw
	;;

  restart)
  	pids=`ps | grep zigbeeHAgw | grep -v grep`
  	if [ "$pids" != "" ]; then
    	killall -TERM zigbeeHAgw
	fi
  	# ./zigbeeHAgw >& /tmp/zigbeeHAgw.out &
  	./zigbeeHAgw &
	;;

  status)
  	# cause zigbeeHAgw to dump server state to /tmp/hagw.servers
	echo asking for status, my PID is $$
  	pids=`ps | grep zigbeeHAgw | grep -v grep`
  	if [ "$pids" = "" ]; then
		echo "Texas Instruments zigbee HA gateway not running"
	else
		gw_status=unknown
  		killall -SIGUSR1 zigbeeHAgw

		echo will sleep waiting for an answer

		# this may seem odd: the first sleep will be 
		# interrupted by the signal. the second sleep
		# gives us time to process the signal
		sleep 1
		echo got past first sleep
		sleep 1
		echo got past second sleep

		echo "Texas Instruments zigbee HA gateway status is "
		case "$gw_status" in

		unknown )
			echo unknown
			;;

		# these are the expected cases
		running )
			echo -n "running since " 
			cat /tmp/tihagw.start
			;;

		started )
			echo -n "started at " 
			cat /tmp/tihagw.start
			;;

		stopped )
			echo at least one TI HA GW server is stopped
			;;

		*)
			echo "unexpected!"
			;;
		esac

		# cat /tmp/hagw.servers
		# /bin/rm -f /tmp/hagw.servers
	fi
	;;

  *)
	echo "Usage: $0 {start|stop|restart|status}"
	exit 1
esac


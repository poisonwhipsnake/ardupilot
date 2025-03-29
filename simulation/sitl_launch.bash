#!/bin/bash
set -x
set -e
set -m

echo "Starting ArduPilot instance"

if [ "${MCAST2}" -eq 1 ]; then
    SITL_MCAST="--uart2 mcast:"
    SERIAL2_OPTIONS="SERIAL2_OPTIONS 1024"
fi

if [ -z "${SYSID}" ]; then
    SYSID=1
fi

if [ -z "${SITL_PARAMETER_LIST}" ]; then
    SITL_PARAMETER_LIST="/ardupilot/rover.parm"
fi

if [ -z "${SITL_LAT}" ]; then
    SITL_LAT="-35.363261"
fi

if [ -z "${SITL_LON}" ]; then
    SITL_LON="149.165230"
fi

if [ -z "${SITL_ALT}" ]; then
    SITL_ALT="584"
fi

if [ -z "${SITL_HEADING}" ]; then
    SITL_HEADING="353"
fi

SITL_LOCATION="$SITL_LAT,$SITL_LON,$SITL_ALT,$SITL_HEADING"

args="--home ${SITL_LOCATION} --model Rover --speedup 1 --defaults ${SITL_PARAMETER_LIST} ${SITL_MCAST}"

echo "args:"
echo "$args"

export MAVLINK20=1
# Start ArduPilot simulator
/ardupilot/ardurover ${args}
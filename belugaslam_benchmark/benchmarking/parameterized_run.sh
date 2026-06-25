#!/bin/bash

SCRIPT_DIR=$(cd $(dirname "$(readlink -f "$0")") && pwd)

read -r -d '' HELP << EOM
Usage: $(basename $0) [...] <PARTICLES_0> ... <PARTICLES_N>\n
\n
    MAX_PARTICLES_N         For each positional argument the benchmark will be run using that amount of particles.\n
    [--package]         Package that OWNS the launch file, defaults to belugaslam_example.\n
    [--executable]      Executable to use, defaults to belugaslam_node.\n
    [--launch-file]     Launch file relative path, defaults to mit_rosbag_belugaslam.xml.\n
    [-b|--rosbag]       Use a different rosbag path, the names of the frames and topics should match with the launch file.\n
    [-r|--playback-rate] Rosbag playback frequency, defaults to 1.0.\n
    [--record-bag]     If set, the benchmark will record a rosbag of the execution in the output folder.\n
    [-h|--help]         Show this help message.
EOM

set +o errexit
VALID_ARGS=$( \
    OPTERR=1 getopt -o b:r:ph --long \
    package:,executable:,launch-file:,rosbag:,playback-rate:,record-bag:,profile,help \
    -- "$@")
RET_CODE=$?
set -o errexit

if [[ $RET_CODE -eq 1 ]]; then
    echo -e "$HELP"
    exit 1;
fi

PACKAGE_NAME="belugaslam_example"
LAUNCH_FILE="mit_rosbag_belugaslam.xml"
EXECUTABLE_NAME="belugaslam_node"
PLAYBACK_RATE="1.0"
ROSBAG_PATH=""
RECORD_BAG=true

eval set -- "$VALID_ARGS"
while : ;do
    case "$1" in
    --package)        PACKAGE_NAME=$2; shift 2 ;;
    --executable)     EXECUTABLE_NAME=$2; shift 2 ;;
    --launch-file)    LAUNCH_FILE=$2; shift 2 ;;
    -b | --rosbag)    ROSBAG_PATH=$2; shift 2 ;;
    -r | --playback-rate) PLAYBACK_RATE=$2; shift 2 ;;
    --record-bag)     RECORD_BAG=true; shift 1 ;;
    -h | --help)      echo -e "$HELP"; exit 0 ;;
    --)               shift; break ;;
    esac
done

if [[ -z "$@" ]]; then
    >&2 echo "Error: At least one number of particles must be specified."
    exit 1
fi

if [[ -n "$ROSBAG_PATH" ]]; then
    if [ ! -d "$ROSBAG_PATH" ] && [ ! -f "$ROSBAG_PATH" ]; then
        >&2 echo "Error: Bag path '$ROSBAG_PATH' not found."
        exit 1
    fi
    ROSBAG_PATH=$(readlink -f "$ROSBAG_PATH")
fi

function cleanup() {
    echo -e "\nTerminating benchmark..."

    kill -SIGINT $(jobs -p) > /dev/null 2>&1
    wait $(jobs -p) > /dev/null 2>&1

    # matar cualquier rosbag que haya quedado vivo
    pkill -f "ros2 bag play" > /dev/null 2>&1 || true
    pkill -f "rosbag2_player" > /dev/null 2>&1 || true
    pkill -f "ros2 bag record" > /dev/null 2>&1 || true
    pkill -f "rosbag2_recorder" > /dev/null 2>&1 || true
}
trap cleanup EXIT ERR

for N in "$@"; do
    FOLDER="bench_output_${N}_particles"
    echo -e "\n>>> RUNNING BENCHMARK WITH $N PARTICLES..."
    mkdir -p "$FOLDER"
    
    STATS_FILE="$(pwd)/$FOLDER/time_stats.txt"
    LOG_FILE="$(pwd)/$FOLDER/console_output.log"
    
    cd "$FOLDER"

    TIME_PREFIX="/usr/bin/time -v -o time_stats.txt"

    script -qefc "$TIME_PREFIX ros2 launch $PACKAGE_NAME $LAUNCH_FILE \
        max_particles:=$N \
        bag_rate:=$PLAYBACK_RATE \
        record_bag:=$RECORD_BAG \
        $( [[ -n "$ROSBAG_PATH" ]] && echo "bag_path:=$ROSBAG_PATH" )" \
        "$LOG_FILE" &

    wait %1

    echo "Computing RMSE..."

    ros2 run belugaslam_benchmark compute_rmse.py \
        "$(pwd)/bag_output_mit" \
        "$(pwd)" \
        --gt-file /home/azul/ros2_ws/src/fastslam_oc_grid/belugaslam_example/bags/mit_rosbag/gt.txt

    echo "Finished $N particles. Results in $FOLDER/"
    cd ..
done

echo -e "\nBenchmark Suite Completed."
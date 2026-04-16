#!/bin/bash
# Benchmarking similar to Beluga's but using GNU Time.

SCRIPT_DIR=$(cd $(dirname "$(readlink -f "$0")") && pwd)

read -r -d '' HELP << EOM
Usage: $(basename $0) [...] <PARTICLES_0> ... <PARTICLES_N>\n
\n
    PARTICLES_N         For each positional argument the benchmark will be run using that amount of particles.\n
    [--package]         Package that OWNS the launch file, defaults to fastslam_example.\n
    [--executable]      Executable to use, defaults to fastslam_node.\n
    [--launch-file]     Launch file relative path, defaults to beluga_rosbag_fastslam.xml.\n
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

# --- CONFIGURACIÓN POR DEFECTO CORREGIDA ---
PACKAGE_NAME="fastslam_example"
LAUNCH_FILE="beluga_rosbag_fastslam.xml"
EXECUTABLE_NAME="fastslam_node"
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

# --- FIX: Convertir ruta de la bag a absoluta antes de hacer 'cd' ---
if [[ -n "$ROSBAG_PATH" ]]; then
    # Si la ruta no existe, avisar antes de fallar
    if [ ! -d "$ROSBAG_PATH" ] && [ ! -f "$ROSBAG_PATH" ]; then
        >&2 echo "Error: Bag path '$ROSBAG_PATH' not found."
        exit 1
    fi
    ROSBAG_PATH=$(readlink -f "$ROSBAG_PATH")
fi

# Función de limpieza al interrumpir
function cleanup() {
    echo -e "\nTerminating benchmark..."
    kill -SIGINT $(jobs -p) > /dev/null 2>&1
    wait $(jobs -p) > /dev/null 2>&1
}
trap cleanup EXIT ERR

# Loop principal
for N in "$@"; do
    FOLDER="bench_output_${N}_particles"
    echo -e "\n>>> RUNNING BENCHMARK WITH $N PARTICLES..."
    mkdir -p "$FOLDER"
    
    # Guardamos la ruta absoluta del archivo de stats
    STATS_FILE="$(pwd)/$FOLDER/time_stats.txt"
    LOG_FILE="$(pwd)/$FOLDER/console_output.log"
    
    cd "$FOLDER"

    # Prefijo de tiempo
    TIME_PREFIX="/usr/bin/time -v -o time_stats.txt"

    # LANZAMIENTO
    script -qefc "$TIME_PREFIX ros2 launch $PACKAGE_NAME $LAUNCH_FILE \
        num_particles:=$N \
        bag_rate:=$PLAYBACK_RATE \
        record_bag:=$RECORD_BAG \
        $( [[ -n "$ROSBAG_PATH" ]] && echo "bag_path:=$ROSBAG_PATH" )" \
        "$LOG_FILE" &

    wait %1

    echo "Finished $N particles. Results in $FOLDER/"
    cd ..
done

echo -e "\nBenchmark Suite Completed."
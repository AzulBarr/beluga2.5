#!/usr/bin/env python3

import os
import subprocess
import argparse
import sys
from ament_index_python.packages import get_package_share_directory

def setup_kernel():
    """Asegura que el kernel permita perf sin sudo."""
    print("Configurando parámetros del kernel para el perfilado...")
    os.system('sudo sysctl -w kernel.perf_event_paranoid=-1')
    os.system('sudo sysctl -w kernel.kptr_restrict=0')

def main():
    parser = argparse.ArgumentParser(description='Replicando el profiler de Beluga para FastSLAM.')
    parser.add_argument('--bag_path', type=str, 
                        default='/home/azul/ros2_ws/src/fastslam_oc_grid/fastslam_example/bags/beluga_rosbag',
                        help='Ruta a la rosbag')
    parser.add_argument('--rate', type=float, default=1.0, help='Velocidad de reproducción')
    args = parser.parse_args()

    # 1. Configurar Kernel
    setup_kernel()

    # 2. Definir ruta de salida para los datos
    output_file = os.path.join(os.getcwd(), "perf.data")

    # 3. Definir el prefijo de PERF (igual al de Beluga)
    # -g: call-graph, dwarf: modo de depuración, -F 99: frecuencia
    perf_prefix = f"perf record -e cycles -F 99 -g --call-graph dwarf -o {output_file} --"

    # 4. Obtener el share del paquete para encontrar el XML
    pkg_share = get_package_share_directory('fastslam_example')
    launch_file = os.path.join(pkg_share, 'example', 'launch', 'beluga_rosbag_fastslam.xml')

    # 5. Construir el comando final
    cmd = [
        'ros2', 'launch', 'fastslam_example', 'beluga_rosbag_fastslam.xml',
        f'bag_path:={args.bag_path}',
        f'slam_prefix:={perf_prefix}',
        'use_sim_time:=true'
    ]

    print(f"\n--- Iniciando perfilado estilo Beluga ---")
    print(f"Salida: {output_file}\n")

    try:
        subprocess.run(cmd)
    except KeyboardInterrupt:
        pass
    
    print(f"\n--- Perfilado terminado. Datos guardados en {output_file} ---")

if __name__ == '__main__':
    main()
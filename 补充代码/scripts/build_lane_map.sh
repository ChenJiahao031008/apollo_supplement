#! /usr/bin/env bash

set -e

TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"

echo "[INFO] TOP_DIR is " $TOP_DIR

bash supplement/update_docker.sh
echo "[INFO] Installation Dependency Finished."

./bazel-bin/modules/tools/map_gen/extract_path path.txt $1/*

./bazel-bin/modules/tools/map_gen/map_gen_single_lane path.txt base_map.txt 3

mkdir -p modules/map/data/$2
cp base_map.txt modules/map/data/$2

./bazel-bin/modules/tools/create_map/convert_map_txt2bin \
    -i $TOP_DIR/modules/map/data/$2/base_map.txt \
    -o $TOP_DIR/modules/map/data/$2/base_map.bin


bash scripts/generate_routing_topo_graph.sh \
    --map_dir $TOP_DIR/modules/map/data/$2

./bazel-bin/modules/map/tools/sim_map_generator \
    --map_dir=$TOP_DIR/modules/map/data/$2 \
    --output_dir=$TOP_DIR/modules/map/data/$2

echo "[INFO] Lane Build Finished."

bash $TOP_DIR/supplement/msf_map_creator.sh \
    $1 \
    $TOP_DIR/modules/calibration/data/dev_kit_pix_hooke/lidar_params/lidar16_novatel_extrinsics.yaml \
    50 \
    $TOP_DIR/modules/map/data/$2/local_map \
    lidar16

bash $TOP_DIR/supplement/ndt_map_creator.sh \
    $1 \
    $TOP_DIR/modules/calibration/data/dev_kit_pix_hooke/lidar_params/lidar16_novatel_extrinsics.yaml \
    50 \
    $TOP_DIR/modules/map/data/$2/ndt_map \
    lidar16

# bash $TOP_DIR/supplement/msf_map_creator.sh \
#     $1 \
#     $TOP_DIR/modules/calibration/data/Lincoln2017MKZ_LGSVL/velodyne_params/velodyne128_novatel_extrinsics.yaml \
#     10 \
#     $TOP_DIR/modules/map/data/$2/local_map \
#     lidar128

# bash $TOP_DIR/supplement/ndt_map_creator.sh \
#     $1 \
#     $TOP_DIR/modules/calibration/data/Lincoln2017MKZ_LGSVL/velodyne_params/velodyne128_novatel_extrinsics.yaml \
#     10 \
#     $TOP_DIR/modules/map/data/$2/ndt_map \
#     lidar128


echo "[INFO] NDT/MSF Map Build Finished."

rm -rf $TOP_DIR/path.txt $TOP_DIR/base_map.txt

echo "[INFO] All Finished."


rtv_root=/media/psf/Home/Documents/data/ca_shuai/2018-03-15/test
exe_addr=/media/psf/Home/Documents/code/dev/core/algorithm_vehicle_localization/examples/RTV2BAG/rtv_bag

for f in $(find ${rtv_root} -name '*.rtv')
do
    rtv_path=$(dirname ${f})
    rtv_root=$(dirname ${rtv_path})
    rtv_name=$(basename ${rtv_path})
    echo ${rtv_root}
    echo ${rtv_name}
    ${exe_addr} ${rtv_root} ${rtv_name}
done

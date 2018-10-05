rtv_root=/media/psf/Home/Documents/data/ca_alex
exe_addr=/media/psf/Home/Documents/code/dev/core/algorithm_vehicle_localization/examples/RTV2BAG/rtv_bag

for f in $(find ${rtv_root} -name '*.rtv')
do 
   rtv_path=$(dirname ${f})
   rtv_name=$(basename ${f})
   echo ${rtv_name}
done

ulimit -c unlimited
exe_addr=/media/psf/Home/Documents/code/dev/core/algorithm_vehicle_localization/examples/DBEstimtor/DBEstimtor
config_adddr=/media/psf/Home/Documents/code/dev/core/algorithm_vehicle_localization/config/slamConfig_loc.json
db_addr=/media/psf/Home/Documents/data/ca_slam/SlamSnippet_0_0.625000
${exe_addr} ./bag_list.txt ${config_adddr} ${db_addr}


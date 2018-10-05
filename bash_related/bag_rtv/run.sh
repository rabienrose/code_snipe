bag_root=/media/psf/Home/Documents/data/ca_alex/sunny/l_r/lane1
exe_addr=/home/test/Documents/bag_rtv
bag_name=2018-01-20_T_18-47-18.731_GMT
#gdb --args ${exe_addr} ${bag_root} ${bag_name}
LD_LIBRARY_PATH=/home/test/Documents ${exe_addr} ${bag_root} ${bag_name}


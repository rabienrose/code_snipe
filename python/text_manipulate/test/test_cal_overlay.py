import common.helper
raw_gps_addr = '/Volumes/chamo/dataset/raw_gps/test/'
# traj1 = 'gps_2016-11-01_T_13-09-25.791_GMTcut1.kml'
# traj2 = 'gps_2016-11-01_T_15-34-09.644_GMTcut2.kml'
traj1 = 'gps_2016-11-02_T_15-45-37.824_GMTcut3.kml'
traj2 = 'gps_undistort_2016-11-01_T_13-08-58.390_GMT_3.kml'
stand_point=[]
traj1_list, stand_point= common.helper.read_traj(raw_gps_addr+traj1, stand_point)
traj2_list, stand_point= common.helper.read_traj(raw_gps_addr+traj2, stand_point)
overlay = common.helper.cal_overlay(traj1_list, traj2_list)
print(overlay)
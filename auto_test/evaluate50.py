# -*- coding: utf-8 -*-

import os
from datetime import datetime

def loc_result_evaluate(compareResult_txt_path,evaluate_result_txt_path):
    deviation50 = {}
    aver_list=[]
    with open(compareResult_txt_path, 'r') as compareResult:
        for line in compareResult.readlines():
            r=float(line.split()[1])
            aver_list.append(r)
    aver_value=float(sum(aver_list) / len(aver_list))
    aver_value=("%.3f" % aver_value)

    with open(compareResult_txt_path, 'r') as compareResult:
        for line in compareResult.readlines():
            r1=float(line.split()[0])
            r2=float(line.split()[1])
            if r2 > 0.5:
                deviation50[r2]=r1
    #deviation50_txt_path=compareResult_txt_path.strip(".txt")+"deviation50_"+datetime.now().strftime('%y-%m-%d_%I:%M:%S_%p')+ '.txt'
    deviation50_txt_path=evaluate_result_txt_path+'/output_for_single_compareResult'+'/'+compareResult_txt_path.split('/')[-1]
    keys=deviation50.keys()
    totalFrameFordeviation50 = float(len(keys)) / float(len(aver_list))
    totalFrameFordeviation50 = ("%.3f" % totalFrameFordeviation50)
    #average_devitation=float(sum(keys)) / len(keys)

    keys.sort(reverse = True)

    with open(deviation50_txt_path, 'w') as deviation50_txt:
        deviation50_txt.write("average_value:"+str(aver_value)+'\n')
        deviation50_txt.write("totalFrameFordeviation50:"+str(totalFrameFordeviation50)+'\n')

        deviation50_txt.write("*********frame & deviation:*********"+'\n')

        for key in keys:
            deviation50_txt.write(str(deviation50[key])+":"+str(key))
            deviation50_txt.write('\n')
    case_info=compareResult_txt_path.split('/')[-1].split('db:')[0]+'db'
    case_name=case_info.split('_T_')[1].split('_db')[0]
    db_info=compareResult_txt_path.split('/')[-1].split('db:')[1]
    #db_name=db_info.split('_T_')[1].split('_GMT')[0]
    db_name=db_info.split('_compare.txt')[0]
    summary_evaluate(evaluate_result_txt_path,case_name,db_name,str(aver_value),str(totalFrameFordeviation50))

def summary_evaluate(summary_evaluate_result,case_name,db_name,average_value,totalFrameFordeviation50):
        with open(summary_evaluate_result+'/summary_evaluate_result.txt', 'a+') as summary_evaluate_result_txt:
            summary_evaluate_result_txt.write(case_name+'  '+db_name+'    '+average_value+'       '+totalFrameFordeviation50+'\n')


def evaluate_all(compare_path,evaluate_result_txt_path):
    print(compare_path)
    print(evaluate_result_txt_path)
    with open(evaluate_result_txt_path + '/summary_evaluate_result.txt', 'w') as summary_evaluate_result_txt:
        summary_evaluate_result_txt.write('case_name                     db_name                       average_value      totalFrameFordeviation50'+'\n')
    compare_folder= os.listdir(compare_path)
    os.mkdir(evaluate_result_txt_path+'/output_for_single_compareResult')
    for compareResult_txt in compare_folder:
        compareResult_txt_path = os.path.join(compare_path, compareResult_txt)
        loc_result_evaluate(compareResult_txt_path,evaluate_result_txt_path)


#compareResult_txt_path='/media/psf/Untitled/DatacenterForAutoTest_20180328/tool_test_0330/compare'
#evaluate_result_txt_path='/media/psf/Untitled/DatacenterForAutoTest_20180328/tool_test_0330/evaluate_result'
#evaluate_all(compareResult_txt_path,evaluate_result_txt_path)

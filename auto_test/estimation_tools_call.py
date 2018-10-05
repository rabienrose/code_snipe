import os
import string
import commands
import shutil


def using_estimation_tools(loc_result_path,task_folder_by_name_path,estimation_output_path):
    if os.path.exists(estimation_output_path):
       shutil.rmtree(estimation_output_path)
    os.mkdir(estimation_output_path)
    plugin_estimation_path=task_folder_by_name_path+'/plugin_estimation'
    for estimation_tool in os.listdir(plugin_estimation_path):
	estimation_tool_input_txt_path=os.path.join(plugin_estimation_path,estimation_tool+'/input/'+estimation_tool+'_input.txt')
        values=[]
        with open(estimation_tool_input_txt_path, 'r') as estimation_tool_input_txt:
	     for line in estimation_tool_input_txt.readlines():
	         values.append(line.split('\n')[0])
	try:     
	    cmd='ls '+task_folder_by_name_path+'/'+values[0]  
	    os.system(cmd)
	except Exception as error:
            print error
	estimation_func_by_diff_db(values,loc_result_path,task_folder_by_name_path,estimation_output_path)


def estimation_func_by_diff_db(values,loc_result_path,task_folder_by_name_path,estimation_output_path):
    for single_db in os.listdir(loc_result_path):
	single_db_path=os.path.join(loc_result_path,single_db)
	estimation_func(values,single_db_path,task_folder_by_name_path,estimation_output_path)


def estimation_func(values,single_db_path,task_folder_by_name_path,estimation_output_path):
    db_name=single_db_path.split('/')[-1]
    case_name_list=os.listdir(single_db_path)
    for case_name in case_name_list:
        case_loc_result_path=os.path.join(single_db_path,case_name)
        estimation_tool_func(values,case_loc_result_path,db_name,task_folder_by_name_path,estimation_output_path)



def estimation_tool_func(values,case_loc_result_path,db_name,task_folder_by_name_path,estimation_output_path):
    estimation_tool_name=values[0].split('/')[-1]
    if len(values) < 2:
       print 'estimation_tool: '+ estimation_tool_name +' input.txt config wrong!please check it!!'
       return
    exe_path=task_folder_by_name_path+'/'+values[0]
    if os.path.exists(estimation_output_path+'/temp_folder'):
       shutil.rmtree(estimation_output_path+'/temp_folder')
    os.mkdir(estimation_output_path+'/temp_folder')
    for i in range(1,len(values)):
        for root, dirs, files in os.walk(case_loc_result_path):
            for file in files:
                if values[i] in file:
                   input_file_path=os.path.join(root,file)
		   input_param_folder_path=estimation_output_path+'/temp_folder/input_param_folder_'+str(i)
                   os.mkdir(input_param_folder_path)
                   print input_param_folder_path
                   shutil.copy(input_file_path,input_param_folder_path)
    if not os.listdir(estimation_output_path+'/temp_folder'):
       shutil.rmtree(estimation_output_path+'/temp_folder')
       return
    cmd=exe_path
    for i in range(1,len(values)):
        k=''
	for j in os.listdir(estimation_output_path+'/temp_folder/input_param_folder_'+str(i)):
  	    k=j
        cmd+=' '+estimation_output_path+'/temp_folder/input_param_folder_'+str(i)+'/'+j
    case_name=case_loc_result_path.split('/')[-1]
    case_db_folder='case_'+case_name+'_db_'+db_name
    case_db_folder_path=estimation_output_path+'/'+case_db_folder
    if not os.path.exists(case_db_folder_path):
       os.mkdir(case_db_folder_path)
    estimation_tool_name_path=case_db_folder_path+'/'+estimation_tool_name
    os.mkdir(estimation_tool_name_path)
    try:
        os.chdir(estimation_tool_name_path)
        #os.system(cmd)
        value=commands.getstatusoutput(cmd)
        with open(estimation_tool_name_path+'/kml_estimation_result.txt', 'w') as kml_estimation_txt:
	     kml_estimation_txt.write(case_name.split('_T_')[1]+' '+ db_name +' '+str(value[1])+ '\n')
    except Exception as error:
       #print error
        pass
    shutil.rmtree(estimation_output_path+'/temp_folder')

if __name__ == '__main__':
    loc_result_path='/home/user/Documents/hyf_task_folder/datacenter/output/loc_result'
    task_folder_by_name_path='/home/user/Documents/hyf_task_folder'
    estimation_output_path='/home/user/Documents/hyf_task_folder/datacenter/output/estimation_output'
    using_estimation_tools(loc_result_path,task_folder_by_name_path,estimation_output_path)





	    

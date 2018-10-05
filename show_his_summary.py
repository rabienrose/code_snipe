import os
import shutil
import random

str_root="/Users/user/Documents/data/loc_re_tesst"

figure_temp='<html>\n' \
'<head>\n' \
'<title>Title of the document</title>\n' \
'<script src="Chart.js"></script>\n' \
'</head>\n' \
'<body>\n' \
'<div style="width:75%;">\n' \
'<canvas id="myChart"></canvas>\n' \
'</div>\n' \
'<script>\n' \
'new Chart(document.getElementById("myChart"),\n' \
'         {\n' \
'          type: "line",\n' \
'          data: {\n' \
'          labels: ["{time_1}", "{time_2}", "{time_3}", "{time_4}"],\n' \
'          datasets: [{data}          ]\n' \
'          },\n' \
'          options: {\n' \
'		        animation: {\n' \
'          duration: 0,\n' \
'          },\n' \
'          hover: {\n' \
'          animationDuration: 0,\n' \
'          },\n' \
'          responsiveAnimationDuration: 0,\n' \
'          layout: {\n' \
'          padding: {\n' \
'          left: 100,\n' \
'          right: 100,\n' \
'          top: 100,\n' \
'          bottom: 100\n' \
'          }},}});\n' \
'</script>\n' \
'</body>\n' \
'</html>'

data_temp='{label: "{case_name}",data: {data},fill: false,borderColor: "rgb({r}, {g}, {b})",lineTension: 0.1}'

def genWebSummary(time_list, data):
    re=figure_temp.replace("{time_1}", time_list[0])
    re=re.replace("{time_2}", time_list[1])
    re=re.replace("{time_3}", time_list[2])
    re=re.replace("{time_4}", time_list[3])
    data_all="\n"
    for case in data:
    	  temp_data=data_temp.replace("{case_name}", case["case_name"])
    	  r = random.randint(0, 255)
    	  g = random.randint(0, 255)
    	  b = random.randint(0, 255)
    	  temp_data=temp_data.replace("{r}", str(r))
    	  temp_data=temp_data.replace("{g}", str(g))
    	  temp_data=temp_data.replace("{b}", str(b))
    	  data_str="[";
    	  for value in case["case_data"]:
    	  	  data_str=data_str+str(value)+","
    	  data_str=data_str+"]"
    	  temp_data=temp_data.replace("{data}", data_str)
    	  data_all=data_all+"                     "+temp_data+",\n"
    re=re.replace("{data}", data_all)
    file = open("his_summary.html","w") 
    file.write(re) 
    file.close()

def compare(x, y):  
    stat_x = os.stat(str_root + "/" + x)  
    stat_y = os.stat(str_root + "/" + y)  
    if stat_x.st_ctime > stat_y.st_ctime:  
        return -1  
    elif stat_x.st_ctime < stat_y.st_ctime:  
        return 1  
    else:  
        return 0  

task_list = os.listdir(str_root)
task_list.sort(compare) 
#print(task_list)
time_list=[]
data_raw={}
case_list=[]
for file_name in task_list:
    if file_name.find("all_result")==-1:
        continue
    txt_file_addr=str_root+"/"+file_name+"/evaluate_output/kml_estimation_result.txt"
    if os.path.isfile(str_root+"/"+file_name):
        continue
    splied1=file_name.split("_")
    time_day=splied1[2]
    time_hour=splied1[3]
    time_list.append(time_day.split("-")[2]+"-"+time_hour)
    file = open(txt_file_addr, "r") 
    temp_case_list=[]
    case_value={}
    for line in file:
        if line.find(".")==-1:
            continue
        splited = line.split()
        temp_case_list.append(splited[0])
        case_value[splited[0]]=splited[2]
    if len(case_list)==0:
       case_list=temp_case_list
       for case in case_list:
           data_raw[case]=[]
    for case in case_list:
        if case_value.has_key(case):
            data_raw[case].append(case_value[case])
        else:
            data_raw[case].append(0)
            
data=[]
for key in data_raw:
    data_case={}
    data_case["case_name"]=key
    data_case["case_data"]=[]
    for value in data_raw[key]:
        data_case["case_data"].append(value)
    data.append(data_case)

genWebSummary(time_list, data)


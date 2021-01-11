import requests
import json
import base64
import time
import os
import numpy as np

def get_time_stamp():
    ct = time.time()
    local_time = time.localtime(ct)
    data_head = time.strftime("%Y-%m-%d %H:%M:%S", local_time)
    data_secs = (ct - int(ct)) * 1000
    time_stamp = "%s.%03d" % (data_head, data_secs)
    print(time_stamp)
get_time_stamp()
#print (time.strftime('%H:%M:%S',time.localtime(time.time())))
for  root, dir, files in os.walk("./data"):
    print( root, dir, files)
    for file in files:
        print(file)
        #with open("./wangyanan/" + str(i) + ".JPG", 'rb') as f:
        with open("./data/" + file, 'rb') as f:
            encode_img = base64.b64encode(f.read())
            #file_ext = os.path.splitext("./test1.jpg")[1]
            #print('data:image/{};base64,{}'.format(file_ext[1:], encode_img.decode()))
            f.close() 
       
        #url="http://124.70.93.59:7001/users"
        url="http://192.168.0.169:7002/users"
        parms = {
            'img_data': encode_img,  
            'img_type': 'jpg'
        }
         
        headers = {
            'User-agent': 'none/ofyourbusiness',
            'Spam': 'Eggs'
        }
         
        res = requests.post(url, data=parms,headers=headers)  
        get_time_stamp()
        #print (time.strftime('%H:%M:%S',time.localtime(time.time())))
        text = res.text
        print(text)
        #img_dict = json.loads(text)
        #encode_img = img_dict['img_data']


        '''
        with open("./result/"+file, 'wb') as f:
            f.write((base64.b64decode(encode_img)))
            #f.write(encode_img)
            f.close()
        '''
import time
import tkinter as tk
import os
import threading
from stat import S_ISDIR
from tkinter import filedialog
from PIL import Image, ImageTk
import paramiko
import argparse
import getpass

def parse_args():
    parser = argparse.ArgumentParser(description='X-ray Semantic Segmentation Client')
    # model and dataset
    parser.add_argument('--sever_data_dir', type=str,
                        default='/home/test_user03/rilian_code_dont_delete/awesome-semantic-segmentation-pytorch/sever',
                        help='path to the sever data')
    parser.add_argument('--sever_hostname', type=str, default="192.168.89.100", help='sever hostname')
    parser.add_argument('--sever_port', type=int, default=22, help='sever port')
    parser.add_argument('--sever_username', type=str, default="test_user03", help='sever username')
    parser.add_argument('--sever_password', type=str, default="0ULWc+o%", help='sever username')
    parser.add_argument('--local_dir', type=str, default='.\\pred_res', help='path to save the predict result')
    args = parser.parse_args()
    return args

def selectPath():
    # 选择文件path_接收文件地址
    global img_path

    img_path = filedialog.askopenfilename()
    img_path = img_path.replace("/", "\\")

    showPhoto(img_path)

def showPhoto(path):
    global photo
    img = Image.open(path)
    img = img.resize((800, 800))
    photo = ImageTk.PhotoImage(img)
    image_Label = tk.Label(main_box, image=photo)
    image_Label.grid(row=1, column=0, columnspan=9)

def inferImage():
    if not img_path:
        main_entry.delete(0, "end")
        main_entry.insert(0, '请选择图像再点击推理')
        return
    wait_thread = threading.Thread(target=waitResult)
    wait_thread.start()

def waitResult():
    input_dir = args.sever_data_dir+'/input'
    # global done_upload_flag,done_infer_flag, ouput_dir
    done_upload_flag = args.sever_data_dir+'/done_upload'
    done_infer_flag = args.sever_data_dir+'/done_infer'
    ouput_dir = args.sever_data_dir+'/output'
    stdin, stdout, stderr = ssh.exec_command(f"rm -rf {input_dir}; mkdir {input_dir}")
    print('Uploading images')

    # 上传

    main_entry.delete(0, "end")
    main_entry.insert(0, '传输中')
    sftp.put(img_path, input_dir +'/' + os.path.basename(img_path))
    stdin, stdout, stderr = ssh.exec_command(f"mkdir {done_upload_flag}")

    #等待

    main_entry.delete(0, "end")

    main_entry.insert(0,'传输完成,推理中')
    while (1):
        try:
            sftp.stat(done_infer_flag)
            main_entry.delete(0, "end")
            main_entry.insert(0, '推理完成！')
            print("infer done, transfering image...")
            stdin, stdout, stderr = ssh.exec_command(f"rm -rf {done_infer_flag}")
            # waiting = False
            break
        except IOError:
            # if not waiting:
            # print("Done uploading, please wait for sever caculation...", flush=True)

            time.sleep(1)
            # waiting = True

    files = sftp.listdir_attr(ouput_dir)
    all_files = []
    for x in files:
        filename = ouput_dir + '/' + x.filename
        all_files.append(filename)
    local_dir = ''
    for file in all_files:
        local_dir = os.path.join(args.local_dir, os.path.basename(file))
        sftp.get(file, local_dir)
    stdin, stdout, stderr = ssh.exec_command(f"rm -rf {ouput_dir};mkdir {ouput_dir}")
    showPhoto(local_dir)


if __name__=='__main__':
    global args
    args = parse_args()
    # sever_password = getpass.getpass("请输入密码:")
    sever_password = "0ULWc+o%"
    img_path = ''

    while (True):
        try:
            # 创建一个ssh的客户端，用来连接服务器
            ssh = paramiko.SSHClient()
            # 创建一个ssh的白名单
            know_host = paramiko.AutoAddPolicy()
            # 加载创建的白名单
            ssh.set_missing_host_key_policy(know_host)
            ssh.connect(
                hostname=args.sever_hostname,
                port=args.sever_port,
                username=args.sever_username,
                password=args.sever_password
            )

            trans = paramiko.Transport(
                sock=(args.sever_hostname, args.sever_port)
            )

            trans.connect(
                username=args.sever_username,
                password=args.sever_password
            )
            sftp = paramiko.SFTPClient.from_transport(trans)
            break
        except:
            sever_password = getpass.getpass("密码错误，请重新输入:")
            # sever_password = input('密码错误，请重新输入：\n')

    if not os.path.exists(args.local_dir):
        os.makedirs(args.local_dir)
    main_box=tk.Tk()
    main_box.title('缺陷检测系统')
    main_box.geometry("800x880+100+80")

    tk.Button(main_box, text="选择图像", command=selectPath).grid(row=0, column=3)
    tk.Button(main_box, text="模型推理", command=inferImage).grid(row=0, column=5)
    main_entry = tk.Entry(main_box)
    main_entry.grid(row=2, column=0, columnspan=9, rowspan=2)

    showPhoto('firstpage.jpg')
    #变量path

    main_box.mainloop()
    ssh.close()
    sftp.close()

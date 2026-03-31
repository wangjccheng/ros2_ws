import tkinter as tk
import subprocess

def call_service(state):
    # 使用 subprocess 在后台默默执行命令行指令
    cmd = ['ros2', 'service', 'call', '/test/logger', 'std_srvs/srv/SetBool', f"data: {state}"]
    subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    if state == "true":
        status_label.config(text="状态: 🔴 正在录制...", fg="red")
    else:
        status_label.config(text="状态: ⏹️ 已停止录制", fg="green")

# 创建主窗口
root = tk.Tk()
root.title("数据录制控制台")
root.geometry("300x200")
root.eval('tk::PlaceWindow . center') # 窗口居中

# 状态标签
status_label = tk.Label(root, text="状态: 待机", font=("Arial", 14))
status_label.pack(pady=20)

# 开始按钮 (绿色大按钮)
btn_start = tk.Button(root, text="▶ 开始录制", font=("Arial", 16, "bold"), bg="lightgreen", width=15, height=2,
                      command=lambda: call_service("true"))
btn_start.pack(pady=5)

# 停止按钮 (红色大按钮)
btn_stop = tk.Button(root, text="⏹ 停止录制", font=("Arial", 16, "bold"), bg="salmon", width=15, height=2,
                     command=lambda: call_service("false"))
btn_stop.pack(pady=5)

root.mainloop()
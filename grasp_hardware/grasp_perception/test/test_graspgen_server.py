import socket
import pickle
import numpy as np

# 数据准备
point_cloud = np.random.rand(100, 3)  # 模拟点云
array = np.random.rand(7)
payload = {'points': point_cloud, 'array': array}

print(f"Sent pointcloud P with shape {point_cloud.shape}, P[0]: {point_cloud[0]}")
print(f"Sent array A with shape {array.shape}")

# 序列化
data = pickle.dumps(payload)

# 连接服务器
HOST = '10.21.70.145'  # 替换为远程IP
PORT = 50007

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    s.sendall(len(data).to_bytes(8, 'big'))  # 先发送数据长度
    s.sendall(data)  # 再发送数据

    # 接收返回值
    length = int.from_bytes(s.recv(8), 'big')
    recv_data = b''
    while len(recv_data) < length:
        recv_data += s.recv(length - len(recv_data))

    result = pickle.loads(recv_data)
    print("Received result:", result)
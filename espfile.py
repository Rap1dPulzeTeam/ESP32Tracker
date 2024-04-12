import serial.tools.list_ports
import serial
import os

def list_available_ports():
    ports = serial.tools.list_ports.comports()
    print("Available ports:")
    for port in ports:
        print(port.device)

def select_port():
    ports = serial.tools.list_ports.comports()
    print("Available ports:")
    for i, port in enumerate(ports, start=1):
        print(f"{i}. {port.device}")
    while True:
        choice = input("Enter the number of the port you want to use: ")
        try:
            choice = int(choice)
            if 1 <= choice <= len(ports):
                return ports[choice - 1].device
            else:
                print("Invalid choice. Please enter a valid number.")
        except ValueError:
            print("Invalid choice. Please enter a number.")

def list_files():
    # 发送命令以列出文件
    ser.write(b'\xAA\x01A0\x00')
    # 读取串口响应并打印文件列表
    response = ser.readline().decode().strip()
    print("Files in SPIFFS directory:")
    print(response)

def read_file(filename):
    # 发送命令以读取文件
    cmd = b'\xAA' + bytes([len(filename) + 1]) + filename.encode() + b'\x00'
    ser.write(cmd)
    # 读取串口响应并保存文件内容
    response = ser.read()
    with open(filename, 'wb') as f:
        f.write(response)
    print("File", filename, "read from SPIFFS")

def write_file(filename):
    # 发送命令以写入文件
    with open(filename, 'rb') as f:
        data = f.read()
    cmd = b'\xAA' + bytes([len(filename) + 1]) + filename.encode() + b'\x00' + data
    ser.write(cmd)
    print("File", filename, "written to SPIFFS")

def transfer_to_esp(file_path):
    # 将文件传输到ESP32
    with open(file_path, 'rb') as f:
        data = f.read()
    cmd = b'\xAA' + bytes([len(file_path) + 1]) + file_path.encode() + b'\x00' + data
    ser.write(cmd)
    print("File", file_path, "transferred to ESP32")

def transfer_from_esp(filename):
    # 从ESP32传输文件到电脑
    cmd = b'\xAA' + bytes([len(filename) + 1]) + filename.encode() + b'\x00'
    ser.write(cmd)
    data = ser.readall()
    with open(filename, 'wb') as f:
        f.write(data)
    print("File", filename, "transferred from ESP32")

# 列出可用串口供选择
port = select_port()
# 打开选定的串口
ser = serial.Serial(port, 115200, timeout=1)

# 交互式命令行
while True:
    print("1. List files")
    print("2. Read file")
    print("3. Write file")
    print("4. Transfer file to ESP32")
    print("5. Transfer file from ESP32")
    print("6. Exit")
    choice = input("Enter your choice: ")
    if choice == '1':
        list_files()
    elif choice == '2':
        filename = input("Enter filename to read: ")
        read_file(filename)
    elif choice == '3':
        filename = input("Enter filename to write: ")
        write_file(filename)
    elif choice == '4':
        file_path = input("Enter file path to transfer to ESP32: ")
        transfer_to_esp(file_path)
    elif choice == '5':
        filename = input("Enter filename to transfer from ESP32: ")
        transfer_from_esp(filename)
    elif choice == '6':
        break
    else:
        print("Invalid choice. Please enter a valid option.")


import os

current_file_path = os.path.abspath(__file__)
current_dir = os.path.dirname(current_file_path)

print("当前文件绝对路径：", type(current_file_path))
print("当前文件目录：", current_dir)
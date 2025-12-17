#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import subprocess

print("=== 简单模型文件诊断 ===")

# 检查当前目录
current_dir = os.getcwd()
print(f"当前目录: {current_dir}")

# 检查模型文件
model_file = "model/light_det.rknn"
if os.path.exists(model_file):
    print(f"✓ 模型文件存在: {model_file}")
    print(f"  文件大小: {os.path.getsize(model_file)} 字节")
else:
    print(f"✗ 模型文件不存在: {model_file}")

# 检查包路径
try:
    result = subprocess.run(['rospack', 'find', 'rknn_pt'], 
                          capture_output=True, text=True)
    if result.returncode == 0:
        package_path = result.stdout.strip()
        print(f"✓ 包路径: {package_path}")
        
        # 检查包路径下的模型文件
        package_model = os.path.join(package_path, 'model/light_det.rknn')
        if os.path.exists(package_model):
            print(f"✓ 包路径下的模型文件存在: {package_model}")
        else:
            print(f"✗ 包路径下的模型文件不存在: {package_model}")
    else:
        print("✗ 无法找到rknn_pt包")
except Exception as e:
    print(f"✗ 执行rospack时出错: {e}")

# 检查用户目录
home_dir = os.path.expanduser("~")
print(f"用户主目录: {home_dir}")

# 检查可能的路径
possible_paths = [
    "/home/duzhong/dzacs/src/rknn_pt/model/light_det.rknn",
    os.path.join(home_dir, "dzacs/src/rknn_pt/model/light_det.rknn"),
    os.path.join(home_dir, "Desktop/src/rknn_pt/model/light_det.rknn")
]

for path in possible_paths:
    if os.path.exists(path):
        print(f"✓ 找到模型文件: {path}")
        print(f"  文件大小: {os.path.getsize(path)} 字节")
    else:
        print(f"✗ 路径不存在: {path}")

print("\n=== 建议 ===")
print("如果模型文件存在，请检查:")
print("1. 文件权限是否正确")
print("2. RKNN库是否正确安装")
print("3. 环境变量是否正确设置") 
import os
import re

# ================= 配置区域 =================
# 填写你的项目根目录路径 (例如: r"C:\Users\Name\MyProject")
PROJECT_ROOT = r"/home/zzy/ros-ground-control/client"

# 输出文件名
OUTPUT_FILE = "ui_optimization_context.txt"

# 需要忽略的文件夹 (避免扫描依赖库和构建产物)
IGNORE_DIRS = {'.git', 'node_modules', 'dist', 'build', '.idea', '.vscode', '__pycache__'}

# ===========================================

def clean_vue_script(content):
    """
    使用正则表达式去除 <script setup> 标签内的内容，保留标签本身以提示AI这是setup语法。
    同时也处理可能存在的 <script lang="ts" setup> 等变体。
    """
    # 匹配 <script ... setup ...> ... </script>
    # flag=re.DOTALL 让 . 可以匹配换行符
    pattern = r'(<script[^>]*?\bsetup\b[^>]*?>)(.*?)(</script>)'
    
    def replacer(match):
        opentag = match.group(1)
        closetag = match.group(3)
        return f'{opentag}\n  /* 逻辑代码已省略，仅进行UI优化 */\n{closetag}'

    return re.sub(pattern, replacer, content, flags=re.DOTALL)

def get_vue_files(root_path):
    """
    遍历目录，返回所有vue文件的列表 (相对路径)
    """
    vue_files = []
    
    for root, dirs, files in os.walk(root_path):
        # 修改 dirs 列表以原地过滤不需要遍历的文件夹
        dirs[:] = [d for d in dirs if d not in IGNORE_DIRS]
        
        for file in files:
            if file.endswith('.vue'):
                full_path = os.path.join(root, file)
                rel_path = os.path.relpath(full_path, root_path)
                vue_files.append(rel_path)
                
    return sorted(vue_files)

def generate_tree_structure(file_paths):
    """
    根据文件列表生成简单的目录树字符串
    """
    tree_lines = []
    # 提取所有涉及的目录
    dirs = set()
    for path in file_paths:
        parts = path.split(os.sep)
        # 记录每一层级的父目录
        for i in range(len(parts) - 1):
            dirs.add(os.sep.join(parts[:i+1]))
    
    sorted_dirs = sorted(list(dirs))
    
    tree_lines.append("Project Structure (Vue Context Only):")
    tree_lines.append(".")
    
    for d in sorted_dirs:
        depth = d.count(os.sep)
        indent = "  " * (depth + 1)
        basename = os.path.basename(d)
        tree_lines.append(f"{indent}📂 {basename}/")
        
    return "\n".join(tree_lines)

def main():
    if not os.path.exists(PROJECT_ROOT):
        print(f"错误: 路径不存在 - {PROJECT_ROOT}")
        return

    print("正在扫描 Vue 文件...")
    vue_files = get_vue_files(PROJECT_ROOT)
    
    if not vue_files:
        print("未找到任何 .vue 文件。")
        return

    print(f"找到 {len(vue_files)} 个 Vue 文件。正在处理并写入...")

    with open(OUTPUT_FILE, 'w', encoding='utf-8') as out:
        # 1. 写入目录结构
        tree_str = generate_tree_structure(vue_files)
        out.write("=== 1. 项目目录结构 (仅包含UI相关) ===\n")
        out.write(tree_str)
        out.write("\n\n" + "="*50 + "\n\n")

        # 2. 写入每个文件的内容
        out.write("=== 2. 文件代码内容 (逻辑已隐藏) ===\n\n")
        
        for rel_path in vue_files:
            full_path = os.path.join(PROJECT_ROOT, rel_path)
            
            try:
                with open(full_path, 'r', encoding='utf-8') as f:
                    content = f.read()
                
                # 清洗 <script setup> 内容
                cleaned_content = clean_vue_script(content)
                
                # 写入分隔符和文件名
                out.write(f"--- File: {rel_path} ---\n")
                out.write(cleaned_content)
                out.write("\n\n")
                
            except Exception as e:
                print(f"读取文件出错 {rel_path}: {e}")

    print(f"完成！结果已保存至: {os.path.abspath(OUTPUT_FILE)}")

if __name__ == "__main__":
    main()

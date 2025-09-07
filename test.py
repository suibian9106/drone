import re
from pathlib import Path

def extract_ids_from_txt(txt_path):
    """从txt文件中提取消息ID（格式如 "HEARTBEAT (0)"）"""
    ids = set()
    # 正则匹配：消息名称后紧跟 (数字) 的模式，如 "HEARTBEAT (0)" 提取 0
    pattern = r'[A-Z0-9_]+\s+\((\d+)\)'
    with open(txt_path, 'r', encoding='utf-8') as f:
        content = f.read()
        # 找到所有匹配的数字并转换为整数
        matches = re.findall(pattern, content)
        ids.update(int(match) for match in matches)
    return ids

def extract_ids_from_md(md_path):
    """从md表格中提取消息ID（表格列名为“消息ID”）"""
    ids = set()
    # 正则匹配表格中的消息ID行（格式如 "| 0 | HEARTBEAT | ..."）
    # 匹配以 | 开头， followed by 数字，再 followed by | 的模式
    pattern = r'\|\s*(\d+)\s*\|'
    with open(md_path, 'r', encoding='utf-8') as f:
        content = f.read()
        # 找到所有匹配的数字并转换为整数
        matches = re.findall(pattern, content)
        ids.update(int(match) for match in matches)
    return ids

def compare_ids(txt_ids, md_ids):
    """找出仅在txt_ids中存在的ID"""
    only_in_txt = txt_ids - md_ids
    return sorted(only_in_txt)

def compare_ids(txt_ids, md_ids):
    """找出仅在txt_ids中存在的ID"""
    only_in_md = txt_ids - md_ids
    return sorted(only_in_md)

if __name__ == "__main__":
    # 文件路径（根据实际情况修改）
    txt_file = Path("messages.txt")
    md_file = Path("MAVLink消息类型汇总.md")
    
    # 提取ID
    txt_ids = extract_ids_from_txt(txt_file)
    md_ids = extract_ids_from_md(md_file)
    
    # 对比差异
    only_in_txt = compare_ids(txt_ids, md_ids)

    only_in_md = compare_ids(md_ids, txt_ids)
    
    # 输出结果
    print(f"从 {txt_file} 中提取到 {len(txt_ids)} 个消息ID")
    print(f"从 {md_file} 中提取到 {len(md_ids)} 个消息ID")
    if only_in_txt:
        print("\n仅在txt文件中存在的消息ID：")
        print(", ".join(map(str, only_in_txt)))
    elif only_in_md:
        print("\n仅在txt文件中存在的消息ID：")
        print(", ".join(map(str, only_in_md)))
    else:
        print("无差异，所有ID在两个文件中均存在")
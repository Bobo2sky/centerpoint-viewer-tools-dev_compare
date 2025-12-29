#!/usr/bin/env python3
"""从 JSON 文件中提取 terminal_output 并保存为文本文件"""
import json

def extract_terminal_output(json_file='output/evaluation_results.json', 
                            output_file='output/terminal_output.txt'):
    """从 JSON 中提取 terminal_output 并保存为纯文本文件"""
    with open(json_file, 'r', encoding='utf-8') as f:
        data = json.load(f)
    
    if 'terminal_output' in data:
        # 将 terminal_output 写入文本文件
        with open(output_file, 'w', encoding='utf-8') as f:
            f.write(data['terminal_output'])
        
        print(f"✓ 终端输出已提取到: {output_file}")
        print(f"  总字符数: {len(data['terminal_output'])}")
        print(f"  总行数: {len(data['terminal_output'].split(chr(10)))}")
    else:
        print("✗ JSON 文件中没有 terminal_output 字段")

if __name__ == '__main__':
    extract_terminal_output()

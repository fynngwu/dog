#!/usr/bin/env python3
"""
修正MuJoCo XML模型中geom位置，使其与inertial质心位置匹配
"""

import xml.etree.ElementTree as ET
import numpy as np
from pathlib import Path

def parse_vec(s: str) -> np.ndarray:
    if s is None:
        return np.array([0.0, 0.0, 0.0])
    return np.array([float(x) for x in s.split()])

def vec_to_str(v: np.ndarray) -> str:
    return f"{v[0]:.6f} {v[1]:.6f} {v[2]:.6f}"

def fix_body(body_elem, depth=0):
    """递归修正body中的geom位置"""
    name = body_elem.get('name', 'unnamed')
    indent = "  " * depth
    
    # 获取geom信息（排除foot_collision）
    geom_elem = None
    for g in body_elem.findall('geom'):
        if 'foot' not in g.get('name', ''):
            geom_elem = g
            break
    
    # 获取inertial信息
    inertial_elem = body_elem.find('inertial')
    
    if geom_elem is not None and inertial_elem is not None:
        geom_type = geom_elem.get('type', 'sphere')
        geom_pos = parse_vec(geom_elem.get('pos'))
        geom_size = parse_vec(geom_elem.get('size', '0.05'))
        inertial_pos = parse_vec(inertial_elem.get('pos'))
        
        # 计算偏移
        offset = inertial_pos - geom_pos
        
        # 对于capsule，检查是否在范围内
        if geom_type == 'capsule':
            radius = geom_size[0]
            half_length = geom_size[1] if len(geom_size) > 1 else 0.05
            extent = np.array([radius, radius, half_length])
        elif geom_type == 'box':
            extent = geom_size[:3]
        elif geom_type == 'sphere':
            extent = np.array([geom_size[0]] * 3)
        else:
            extent = geom_size[:3] if len(geom_size) >= 3 else np.array([0.05, 0.05, 0.05])
        
        # 检查每个轴
        x_in_range = abs(offset[0]) <= extent[0]
        y_in_range = abs(offset[1]) <= extent[1]
        z_in_range = abs(offset[2]) <= extent[2]
        all_in_range = x_in_range and y_in_range and z_in_range
        
        if not all_in_range:
            # 修正geom位置
            old_pos = geom_elem.get('pos')
            new_pos = vec_to_str(inertial_pos)
            geom_elem.set('pos', new_pos)
            print(f"{indent}修正 {name}:")
            print(f"{indent}  geom位置: {old_pos if old_pos else '[0 0 0]'} -> {new_pos}")
    
    # 递归处理子body
    for child in body_elem.findall('body'):
        fix_body(child, depth + 1)

def main():
    # 解析XML文件
    base_dir = Path(__file__).parent.parent
    tree = ET.parse(base_dir / "leggedrobot_flat.xml")
    root = tree.getroot()
    worldbody = root.find('worldbody')
    
    print("=" * 80)
    print("修正 geom 位置以匹配 inertial 质心")
    print("=" * 80)
    print()
    
    # 修正所有body
    for body_elem in worldbody.findall('body'):
        fix_body(body_elem)
    
    # 保存修正后的文件
    output_path = base_dir / "leggedrobot_flat_fixed.xml"
    tree.write(output_path, encoding='unicode', xml_declaration=True)
    
    print()
    print("=" * 80)
    print(f"修正后的文件已保存到: {output_path}")
    print("=" * 80)

if __name__ == "__main__":
    main()
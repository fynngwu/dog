#!/usr/bin/env python3
"""
详细分析MuJoCo XML模型中geom和inertial的对应关系
检查每个轴的偏差
"""

import xml.etree.ElementTree as ET
import numpy as np
from pathlib import Path

def parse_vec(s: str) -> np.ndarray:
    if s is None:
        return np.array([0.0, 0.0, 0.0])
    return np.array([float(x) for x in s.split()])

def analyze_body(body_elem, depth=0):
    """递归分析body"""
    name = body_elem.get('name', 'unnamed')
    indent = "  " * depth
    
    # 获取geom信息
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
            # capsule沿z轴，所以x,y范围是radius，z范围是half_length
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
            print(f"\n{indent}❌ {name}")
            print(f"{indent}   geom类型: {geom_type}")
            print(f"{indent}   geom位置: [{geom_pos[0]:.6f}, {geom_pos[1]:.6f}, {geom_pos[2]:.6f}]")
            print(f"{indent}   geom尺寸: {geom_size}")
            print(f"{indent}   geom范围: x±{extent[0]:.4f}, y±{extent[1]:.4f}, z±{extent[2]:.4f}")
            print(f"{indent}   inertial质心: [{inertial_pos[0]:.6f}, {inertial_pos[1]:.6f}, {inertial_pos[2]:.6f}]")
            print(f"{indent}   偏移量: [{offset[0]:.6f}, {offset[1]:.6f}, {offset[2]:.6f}]")
            print(f"{indent}   偏移距离: {np.linalg.norm(offset)*1000:.2f} mm")
            print(f"{indent}   轴向检查:")
            print(f"{indent}     X: 偏移 {offset[0]*1000:.2f}mm, 范围 ±{extent[0]*1000:.2f}mm {'✅' if x_in_range else '❌'}")
            print(f"{indent}     Y: 偏移 {offset[1]*1000:.2f}mm, 范围 ±{extent[1]*1000:.2f}mm {'✅' if y_in_range else '❌'}")
            print(f"{indent}     Z: 偏移 {offset[2]*1000:.2f}mm, 范围 ±{extent[2]*1000:.2f}mm {'✅' if z_in_range else '❌'}")
            
            # 计算需要的geom位置
            print(f"{indent}   💡 建议geom位置: [{inertial_pos[0]:.6f}, {inertial_pos[1]:.6f}, {inertial_pos[2]:.6f}]")
        else:
            print(f"{indent}✅ {name}: geom和inertial匹配良好")
    
    # 递归处理子body
    for child in body_elem.findall('body'):
        analyze_body(child, depth + 1)

def main():
    tree = ET.parse(Path(__file__).parent.parent / "leggedrobot_flat.xml")
    root = tree.getroot()
    worldbody = root.find('worldbody')
    
    print("=" * 80)
    print("详细 geom-inertial 对应性分析")
    print("=" * 80)
    print("\n检查每个body的geom是否覆盖了inertial的质心位置:\n")
    
    for body_elem in worldbody.findall('body'):
        analyze_body(body_elem)
    
    print("\n" + "=" * 80)

if __name__ == "__main__":
    main()
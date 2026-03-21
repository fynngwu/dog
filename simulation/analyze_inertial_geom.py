#!/usr/bin/env python3
"""
分析MuJoCo XML模型中geom和inertial的对应关系
检查geom的位置是否合理地覆盖了inertial的质心位置
"""

import xml.etree.ElementTree as ET
import numpy as np
from dataclasses import dataclass
from typing import Optional, List, Tuple
from pathlib import Path

@dataclass
class GeomInfo:
    name: str
    geom_type: str
    pos: np.ndarray
    size: np.ndarray
    rgba: str

@dataclass
class InertialInfo:
    pos: np.ndarray
    mass: float
    inertia: Optional[np.ndarray] = None

@dataclass
class BodyInfo:
    name: str
    pos: np.ndarray
    quat: np.ndarray
    geom: Optional[GeomInfo] = None
    inertial: Optional[InertialInfo] = None
    children: List['BodyInfo'] = None

def parse_vec(s: str) -> np.ndarray:
    """解析空格分隔的向量字符串"""
    if s is None:
        return np.array([0.0, 0.0, 0.0])
    return np.array([float(x) for x in s.split()])

def parse_quat(s: str) -> np.ndarray:
    """解析四元数，默认为单位四元数"""
    if s is None:
        return np.array([1.0, 0.0, 0.0, 0.0])  # w, x, y, z
    return np.array([float(x) for x in s.split()])

def quat_to_rotation_matrix(quat: np.ndarray) -> np.ndarray:
    """将四元数转换为旋转矩阵 (MuJoCo使用w,x,y,z顺序)"""
    w, x, y, z = quat
    
    R = np.array([
        [1 - 2*y*y - 2*z*z, 2*x*y - 2*w*z, 2*x*z + 2*w*y],
        [2*x*y + 2*w*z, 1 - 2*x*x - 2*z*z, 2*y*z - 2*w*x],
        [2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x*x - 2*y*y]
    ])
    return R

def transform_point(point: np.ndarray, pos: np.ndarray, quat: np.ndarray) -> np.ndarray:
    """将点从局部坐标系转换到父坐标系"""
    R = quat_to_rotation_matrix(quat)
    return R @ point + pos

def parse_geom(geom_elem) -> Optional[GeomInfo]:
    """解析geom元素"""
    if geom_elem is None:
        return None
    
    name = geom_elem.get('name', 'unnamed')
    geom_type = geom_elem.get('type', 'sphere' if geom_elem.get('size') else 'plane')
    pos = parse_vec(geom_elem.get('pos'))
    size_str = geom_elem.get('size', '0.05')
    size = parse_vec(size_str)
    rgba = geom_elem.get('rgba', '0.5 0.5 0.5 1')
    
    return GeomInfo(name=name, geom_type=geom_type, pos=pos, size=size, rgba=rgba)

def parse_inertial(inertial_elem) -> Optional[InertialInfo]:
    """解析inertial元素"""
    if inertial_elem is None:
        return None
    
    pos = parse_vec(inertial_elem.get('pos'))
    mass = float(inertial_elem.get('mass', '0'))
    
    inertia = None
    if inertial_elem.get('fullinertia'):
        inertia = parse_vec(inertial_elem.get('fullinertia'))
    elif inertial_elem.get('diaginertia'):
        inertia = parse_vec(inertial_elem.get('diaginertia'))
    
    return InertialInfo(pos=pos, mass=mass, inertia=inertia)

def parse_body(body_elem, depth=0) -> BodyInfo:
    """递归解析body元素"""
    name = body_elem.get('name', 'unnamed')
    pos = parse_vec(body_elem.get('pos'))
    quat = parse_quat(body_elem.get('quat'))
    
    # 解析geom (只取第一个非foot_collision的geom)
    geom = None
    for geom_elem in body_elem.findall('geom'):
        geom_info = parse_geom(geom_elem)
        if 'foot' not in geom_info.name:
            geom = geom_info
            break
    if geom is None and body_elem.find('geom') is not None:
        geom = parse_geom(body_elem.find('geom'))
    
    # 解析inertial
    inertial = parse_inertial(body_elem.find('inertial'))
    
    # 递归解析子body
    children = []
    for child_body in body_elem.findall('body'):
        children.append(parse_body(child_body, depth + 1))
    
    return BodyInfo(name=name, pos=pos, quat=quat, geom=geom, inertial=inertial, children=children)

def get_geom_center_and_extent(geom: GeomInfo) -> Tuple[np.ndarray, np.ndarray]:
    """获取geom的中心和大概范围"""
    center = geom.pos.copy()
    
    if geom.geom_type == 'box':
        # box的size是半尺寸
        extent = geom.size[:3]
    elif geom.geom_type == 'capsule':
        # capsule: size[0]是半径，size[1]是半长度（沿z轴）
        radius = geom.size[0]
        half_length = geom.size[1] if len(geom.size) > 1 else 0.05
        extent = np.array([radius, radius, half_length])
    elif geom.geom_type == 'sphere':
        radius = geom.size[0]
        extent = np.array([radius, radius, radius])
    elif geom.geom_type == 'cylinder':
        radius = geom.size[0]
        half_length = geom.size[1] if len(geom.size) > 1 else 0.05
        extent = np.array([radius, radius, half_length])
    else:
        extent = geom.size[:3] if len(geom.size) >= 3 else np.array([0.05, 0.05, 0.05])
    
    return center, extent

def check_inertial_in_geom(inertial: InertialInfo, geom: GeomInfo) -> dict:
    """检查inertial质心是否在geom范围内"""
    if inertial is None or geom is None:
        return {'valid': False, 'reason': '缺少inertial或geom'}
    
    geom_center, geom_extent = get_geom_center_and_extent(geom)
    inertial_pos = inertial.pos
    
    # 计算inertial相对于geom中心的偏移
    offset = inertial_pos - geom_center
    
    # 检查是否在geom范围内（考虑一定的容差）
    tolerance = 0.01  # 1cm容差
    in_range = np.all(np.abs(offset) <= geom_extent + tolerance)
    
    # 计算距离
    distance = np.linalg.norm(offset)
    
    return {
        'valid': in_range,
        'geom_center': geom_center,
        'geom_extent': geom_extent,
        'inertial_pos': inertial_pos,
        'offset': offset,
        'distance': distance,
        'reason': None if in_range else f"质心偏离geom中心 {distance*1000:.2f}mm"
    }

def analyze_body(body: BodyInfo, parent_transform: Tuple[np.ndarray, np.ndarray] = None, depth=0):
    """递归分析body"""
    if parent_transform is None:
        parent_transform = (np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]))
    
    parent_pos, parent_quat = parent_transform
    
    # 计算当前body在世界坐标系中的位置
    world_pos = transform_point(body.pos, parent_pos, parent_quat)
    
    # 计算当前body的旋转
    # 注意：这里简化处理，实际需要四元数乘法
    
    indent = "  " * depth
    print(f"\n{indent}{'='*60}")
    print(f"{indent}Body: {body.name}")
    print(f"{indent}  局部位置: {body.pos}")
    print(f"{indent}  四元数: {body.quat}")
    
    if body.inertial:
        print(f"{indent}  惯性参数:")
        print(f"{indent}    质心位置: {body.inertial.pos}")
        print(f"{indent}    质量: {body.inertial.mass:.4f} kg")
        if body.inertial.inertia is not None:
            print(f"{indent}    惯性张量: {body.inertial.inertia}")
    else:
        print(f"{indent}  ⚠️ 没有inertial定义!")
    
    if body.geom:
        print(f"{indent}  几何体:")
        print(f"{indent}    类型: {body.geom.geom_type}")
        print(f"{indent}    位置: {body.geom.pos}")
        print(f"{indent}    尺寸: {body.geom.size}")
        
        # 检查geom和inertial的对应关系
        if body.inertial:
            result = check_inertial_in_geom(body.inertial, body.geom)
            print(f"{indent}  对应性检查:")
            print(f"{indent}    geom中心: {result['geom_center']}")
            print(f"{indent}    geom范围: {result['geom_extent']}")
            print(f"{indent}    inertial质心: {result['inertial_pos']}")
            print(f"{indent}    偏移量: {result['offset']}")
            print(f"{indent}    偏移距离: {result['distance']*1000:.2f} mm")
            
            if result['valid']:
                print(f"{indent}    ✅ 质心在geom范围内")
            else:
                print(f"{indent}    ❌ {result['reason']}")
                
                # 建议修正
                suggested_geom_pos = body.inertial.pos
                print(f"{indent}    💡 建议geom位置: {suggested_geom_pos}")
    else:
        print(f"{indent}  ⚠️ 没有geom定义!")
    
    # 递归处理子body
    for child in (body.children or []):
        analyze_body(child, (world_pos, body.quat), depth + 1)

def main():
    # 解析XML文件
    tree = ET.parse(Path(__file__).parent.parent / "leggedrobot_flat.xml")
    root = tree.getroot()
    
    # 找到worldbody
    worldbody = root.find('worldbody')
    
    print("=" * 80)
    print("MuJoCo模型 geom-inertial 对应性分析")
    print("=" * 80)
    
    # 解析所有body
    for body_elem in worldbody.findall('body'):
        body = parse_body(body_elem)
        analyze_body(body)
    
    print("\n" + "=" * 80)
    print("分析总结")
    print("=" * 80)
    
    # 收集所有问题
    issues = []
    
    def collect_issues(body: BodyInfo, path=""):
        path = f"{path}/{body.name}" if path else body.name
        
        if body.geom and body.inertial:
            result = check_inertial_in_geom(body.inertial, body.geom)
            if not result['valid']:
                issues.append({
                    'path': path,
                    'body': body.name,
                    'offset': result['offset'],
                    'distance': result['distance'],
                    'suggested_geom_pos': body.inertial.pos
                })
        
        for child in (body.children or []):
            collect_issues(child, path)
    
    for body_elem in worldbody.findall('body'):
        body = parse_body(body_elem)
        collect_issues(body)
    
    if issues:
        print(f"\n发现 {len(issues)} 个geom-inertial不匹配问题:\n")
        for i, issue in enumerate(issues, 1):
            print(f"{i}. {issue['path']}")
            print(f"   偏移量: {issue['offset']}")
            print(f"   偏移距离: {issue['distance']*1000:.2f} mm")
            print(f"   建议geom位置: {issue['suggested_geom_pos']}")
            print()
    else:
        print("\n✅ 所有geom和inertial位置匹配良好!")

if __name__ == "__main__":
    main()
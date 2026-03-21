import argparse
import numpy as np
import onnxruntime as ort

def test_onnx_inference(model_path):
    # 1. 加载 ONNX 模型
    try:
        session = ort.InferenceSession(model_path)
        print(f"成功加载模型: {model_path}")
    except Exception as e:
        print(f"加载模型失败: {e}")
        return

    # 2. 准备基础的 45 维单帧数据
    raw_data_str = (
        "0.00 -0.00 0.00 -0.00 -0.00 -1.00 0.00 0.00 0.00 0.00 0.00 0.00 "
        "0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 "
        "0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 "
        "0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00"
    )
    # 将字符串以空格分割并转为 float32 的 numpy 数组
    single_obs = np.array([float(x) for x in raw_data_str.split()], dtype=np.float32)
    print(f"提取到单帧观测值，数量: {len(single_obs)} 个")

    # 3. 构造 45 * 10 = 450 的历史观测数据
    # 按照原控制代码逻辑：np.concatenate(hist_obs)[None, :]
    # 这里我们将这 45 维数据存入列表 10 次，模拟 10 帧相同的历史观测值
    hist_obs = [single_obs for _ in range(10)]
    
    # 拼接并增加 Batch 维度 -> 最终形状变为 (1, 450)
    policy_input = np.concatenate(hist_obs)[None, :]
    print(f"已构造模型输入，最终输入形状: {policy_input.shape}")

    # 4. 获取模型的输入节点信息
    input_name = session.get_inputs()[0].name
    
    # 5. 运行推理
    try:
        output = session.run(None, {input_name: policy_input})
        
        # 6. 打印输出结果
        print("\n" + "="*40)
        print("模型推理成功！")
        print("动作输出 (Action):")
        # 打印保留 4 位小数的结果，方便阅读
        np.set_printoptions(precision=4, suppress=True)
        print(output[0])
        print(f"输出形状: {output[0].shape}")
        print("="*40 + "\n")
        
    except Exception as e:
        print(f"推理过程中出错: {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="测试 ONNX 模型推理 (指定45维观测输入)")
    parser.add_argument("--load_model", type=str, required=True, help="ONNX 模型文件的路径")
    
    args = parser.parse_args()
    test_onnx_inference(args.load_model)
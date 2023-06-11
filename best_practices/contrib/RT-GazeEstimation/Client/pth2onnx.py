import timm
import torch
import torch.onnx

def pth2onnx():
    pre_pth = "./data/resnet18/Iter_10_resnet18.pth"

    output_file = "data/resnet18/resnet18.onnx"
    # 模型定义来自于torchvision，样例生成的模型文件是基于resnet50模型
    model = timm.create_model("resnet18", num_classes=2)
    resnet18_model = torch.load(pre_pth, map_location='cpu')
    model.load_state_dict(resnet18_model)

    # 调整模型为eval mode
    model.eval()
    # 输入节点名
    input_names = ["input"]
    # 输出节点名
    output_names = ["output"]
    dynamic_axes = {'input': {0: '-1'}, 'output': {0: '-1'}}
    dummy_input = torch.randn(1, 3, 224, 224)
    # verbose=True，支持打印onnx节点和对应的PyTorch代码行
    torch.onnx.export(model, dummy_input, output_file, input_names = input_names, dynamic_axes = dynamic_axes, output_names = output_names, opset_version=11, verbose=True)

if __name__ == "__main__":
    pth2onnx()
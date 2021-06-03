"""resnet50 classes"""
resnet50_classes = [
"airplane",
"automobile",
"bird",
"cat",
"deer",
"dog",
"frog",
"horse",
"ship",
"truck"
]

def get_resnet50_class(id):
    """get resnet50 classes"""
    if id >= len(resnet50_classes):
        return "unknown"
    else:
        return resnet50_classes[id]

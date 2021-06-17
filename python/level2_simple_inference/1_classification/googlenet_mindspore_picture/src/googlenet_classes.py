"""googlenet classes"""
googlenet_classes = [
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

def get_googlenet_class(id):
    """get googlenet classes"""
    if id >= len(googlenet_classes):
        return "unknown"
    else:
        return googlenet_classes[id]

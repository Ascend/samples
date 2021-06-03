"""
gesture_categories
"""
image_net_classes = [
"0",
"1",
"2",
"3",
"4",
"5",
"6",
"7",
"8",
"9",
"10"
]

def get_image_net_class(image_id):
    """
	get_gesture_categories
    """
    if image_id >= len(image_net_classes):
        return "unknown"
    else:
        return image_net_classes[image_id]
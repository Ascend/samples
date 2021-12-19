"""
gesture_categories
"""
gesture_categories = [
        '0',
        '1',
        '2',
        '3',
        '4',
        '5',
        '6',
        '7',
        '8',
        '9',
        'left',
        'ok',
        'right',
        'rock',
        'finger heart',
        'praise',
        'prayer',
        'stop',
        'Give the middle finger',
        'bow',
        'No gesture'
]

def get_gesture_categories(gesture_id):
    """
	get_gesture_categories
    """
    if gesture_id >= len(gesture_categories):
        return "unknown"
    else:
        return gesture_categories[gesture_id]
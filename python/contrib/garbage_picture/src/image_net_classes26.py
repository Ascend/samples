#encoding=utf8
image_net_classes = [
   "Seashel", "Lighter","Old Mirror", "Broom","Ceramic Bowl", "Toothbrush","Disposable Chopsticks","Dirty Cloth",
     "Newspaper", "Glassware", "Basketball", "Plastic Bottle", "Cardboard","Glass Bottle", "Metalware", "Hats", "Cans", "Paper",
      "Vegetable Leaf","Orange Peel", "Eggshell","Banana Peel",
    "Battery", "Tablet capsules","Fluorescent lamp", "Paint bucket"]


def get_image_net_class(id):
    if id >= len(image_net_classes):
        return "unknown"
    else:
        return image_net_classes[id]

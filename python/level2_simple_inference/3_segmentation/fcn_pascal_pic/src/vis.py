import numpy as np 
def make_palette(num_classes): 
    palette = np.zeros((num_classes, 3), dtype=np.uint8) 
    for k in range(0, num_classes): 
        label = k 
        i = 0 
        while label: 
            palette[k, 0] |= (((label >> 0) & 1) << (7 - i)) 
            palette[k, 1] |= (((label >> 1) & 1) << (7 - i)) 
            palette[k, 2] |= (((label >> 2) & 1) << (7 - i)) 
            label >>= 3 
            i += 1 
    return palette  
 
 
def color_seg(seg, palette): 
    return palette[seg.flat].reshape(seg.shape + (3,)) 
 
 
def vis_seg(img, seg, palette, alpha=0.5): 
    vis = np.array(img, dtype=np.float32) 
    mask = seg > 0 
    vis[mask] *= 1. - alpha 
    vis[mask] += alpha * palette[seg[mask].flat] 
    vis = vis.astype(np.uint8) 
    return vis 

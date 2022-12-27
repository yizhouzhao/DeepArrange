# utils

def is_overlap(x1, y1, x2, y2):
    return min(y1, y2) > max(x1, x2)

def is_boxes_overlap(boxes1, boxes2):
    box1_min, box1_max = boxes1[0], boxes1[1]
    box2_min, box2_max = boxes2[0], boxes2[1]
    
    return is_overlap(box1_min[0], box1_max[0], box2_min[0], box2_max[0]) and \
            is_overlap(box1_min[1], box1_max[1], box2_min[1], box2_max[1]) and \
                is_overlap(box1_min[2], box1_max[2], box2_min[2], box2_max[2]) 
'''
@Time          : 20/04/25 15:49
@Author        : huguanghao
@File          : demo.py
@Noice         :
@Modificattion :
    @Author    : RTU
    @Time      : 20/07/01 20:01
    @Detail    : demo.py
'''

import cv2
import rospkg
from tool.torch_utils import *
from tool.darknet2pytorch import Darknet

'''
@Modificattion :
@Author    : RTU
@Time      : 20/07/01 20:01
@Detail    : utils.py
'''
def load_class_names(namesfile):
    class_names = []
    with open(namesfile, 'r') as fp:
        lines = fp.readlines()
    for line in lines:
        line = line.rstrip()
        class_names.append(line)
    return class_names

def plot_boxes_cv2(img, boxes, class_names=None, color=None):
    colors = np.array([[1, 0, 1], [0, 0, 1], [0, 1, 1], [0, 1, 0], [1, 1, 0], [1, 0, 0]], dtype=np.float32)

    def get_color(c, x, max_val):
        ratio = float(x) / max_val * 5
        i = int(math.floor(ratio))
        j = int(math.ceil(ratio))
        ratio = ratio - i
        r = (1 - ratio) * colors[i][c] + ratio * colors[j][c]
        return int(r * 255)

    width = img.shape[1]
    height = img.shape[0]

    for i in range(len(boxes)):
        box = boxes[i]
        x1 = int((box[0] - box[2] / 2.0) * width)
        y1 = int((box[1] - box[3] / 2.0) * height)
        x2 = int((box[0] + box[2] / 2.0) * width)
        y2 = int((box[1] + box[3] / 2.0) * height)

        if color:
            rgb = color
        else:
            rgb = (255, 0, 0)
        if len(box) >= 4 and class_names:
            cls_conf = box[5]
            cls_id = box[6]
            #print('%s: %f' % (class_names[cls_id], cls_conf))
            classes = len(class_names)
            offset = cls_id * 123457 % classes
            red = get_color(2, offset, classes)
            green = get_color(1, offset, classes)
            blue = get_color(0, offset, classes)
            if color is None:
                rgb = (red, green, blue)
            img = cv2.putText(img, class_names[cls_id], (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1.2, rgb, 3)
        img = cv2.rectangle(img, (x1, y1), (x2, y2), rgb, 3)
    return img

use_cuda = True

path = rospkg.RosPack().get_path('sakura_vision')
print('Reading path ... : %s'%(path))

print('Init darknet ...'),
m = Darknet(path + '/scripts/yolo/cfg/tel_train_ft.cfg')
print('Done!')

print('Loading weights ...'),
m.load_weights(path + '/scripts/yolo/weight/yolov4.weights')
print('Done!')

print('Read names ...'),
class_names = load_class_names(path + '/scripts/yolo/data/tel.names')
print('Done!')

if use_cuda:
    m.cuda()

def detect(img):
    img_sized = cv2.resize(img, (m.width, m.height))
    img_sized = cv2.cvtColor(img_sized, cv2.COLOR_BGR2RGB)
    boxes = do_detect(m, img_sized, 0.78, 0.4, use_cuda)

    
    return boxes[0],plot_boxes_cv2(img, boxes[0], class_names=class_names)
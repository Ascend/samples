# coding=utf-8
import numpy as np
import os
import codecs
from tokenizer import Tokenizer
import time
import json
import sys

path = os.path.dirname(os.path.abspath(__file__))

sys.path.append(os.path.join(path, ".."))
sys.path.append(os.path.join(path, "../../../../common/"))
sys.path.append(os.path.join(path, "../../../../common/atlas_utils"))

from acl_resource import AclResource
from acl_model import Model
from utils import display_time

model_path = '../model/bert_text_classification.om'
dict_path = '../scripts/vocab.txt'
label_path = '../scripts/label.json'
sample_path = '../data/sample.txt'
output_dir = '../out/'
maxlen = 300

token_dict = {}
with codecs.open(dict_path, 'r', 'utf-8') as reader:
    for line in reader:
        token = line.strip()
        token_dict[token] = len(token_dict)

class OurTokenizer(Tokenizer):
    def _tokenize(self, text):
        R = []
        for c in text:
            if c in self._token_dict:
                R.append(c)
            else:
                R.append('[UNK]')   # The remaining characters are [UNK]
        return R

def save_to_file(file_name, contents):
    fh = open(file_name, 'w')
    fh.write(contents)
    fh.close()

def preprocess(text):
    
    tokenizer = OurTokenizer(token_dict)

    # tokenize
    text = text[:maxlen]
    x1, x2 = tokenizer.encode(first=text)

    X1 = x1 + [0] * (maxlen-len(x1)) if len(x1) < maxlen else x1
    X2 = x2 + [0] * (maxlen-len(x2)) if len(x2) < maxlen else x2
    return X1, X2

def postprocess(result_list):
    
    print(result_list[0]) 
    y = np.argmax(result_list[0])
    return y
    
    
def main():
    """
    acl resource initialization
    """
    
     #ACL resource initialization    
    acl_resource = AclResource()
    acl_resource.init()
    
    model = Model(model_path)
    with open(sample_path, "r") as f:
        text = f.read() 
    #text = "餐具用洗涤剂合格不足五成建议：小企业生产的散装产品慎买2006年第一季度，广东省质量技术监督局抽查了广州、深圳、东莞、佛山等6个市企业生产的餐具用洗涤剂产品共29批次，抽样合格率48%，质量问题突出。从抽查结果看，大中型企业和知名品牌产品质量稳定。质量较差的是小型企业生产的散装产品。8批次产品甲醛含量超标。甲醛含量是餐具用洗涤剂的卫生指标之一，甲醛的除菌能力较强，但对人眼、鼻、喉、黏膜有刺激作用，含量超标会危害人体健康。国家标准要求甲醛含量≤0.1mg/g，而不合格产品的该项实测值分别在0.14～0.69mg/g之间，最高的超过标准要求近7倍。这主要是一些企业为了延长保质期，加大甲醛的量；计量器具不准确导致配料称量不准；不重视生产管理，对生产工艺控制不力等原因造成的。7批次产品的总活性物含量不合格。总活性物含量是反映餐具用洗涤剂中表面活性剂多少的指标，加入总活性物量的多少，直接影响餐具用洗涤剂除污垢的效果。国家标准要求总活性物含量≥15％，不合格产品的实测值在5％～13％之间。生产企业为了降低成本而减少产品中表面活性剂的含量，是导致该指标不合格的主要原因。10批次产品的去污力低于标准要求。活性物含量越高，产品的洗涤效能相对越强。产品不合格的主要原因是：一些企业为了获利，减少表面活性剂添加量，以此降低产品成本。1批次产品菌落总数不合格。菌落总数超标，将会对人体造成很大危害。如果人手上有破损时，接触含大量细菌的餐具用洗涤剂，细菌会入侵伤口，使伤口发炎，严重的会引起败血症。检查发现，厂家消毒设备不完善、生产环境的卫生条件不符合要求造成了菌落总数超标。6批次产品标识不合格，主要是缺产品主要成分、生产许可证号、卫生许可证号等。针对抽查发现的质量问题，广东省质监局已采取措施，加大处理力度。责令产品不合格企业限期整改；对产品质量问题严重、存在以次充好行为的依法立案查处，同时向媒体曝光；召开产品质量分析会，宣传有关法律法规以及相关产品标准，分析质量问题及其原因，引导和督促企业增强质量意识，全面提升产品质量。"
    with open(label_path, "r", encoding="utf-8") as f:
        label_dict = json.loads(f.read())
               
    X1, X2 = preprocess(text)
    
    X1 = np.ascontiguousarray(X1, dtype='float32')
    X2 = np.ascontiguousarray(X2, dtype='float32')

    print(X1)
    print(X2)
    X1 = np.expand_dims(X1, 0)
    X2 = np.expand_dims(X2, 0)
    s_time = time.time()
    
    result_list = model.execute([X1, X2])
    e_time = time.time()    
    print(result_list)   
    y = postprocess(result_list)

    if not os.path.exists(output_dir):
        os.mkdir(output_dir)
    save_to_file(output_dir+'prediction_label.txt', label_dict[str(y)])
    print("Original text: %s" % text)
    print("Prediction label: %s" % label_dict[str(y)])
    
    print("Cost time:", e_time-s_time)
    print("Execute end")
    
if __name__ == '__main__':
    main()

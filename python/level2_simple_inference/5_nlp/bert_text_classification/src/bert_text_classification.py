"""
bert text classification
"""
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


class OurTokenizer(Tokenizer):
    """
    tokenizer text to ID
    """
    def _tokenize(self, text):
        R = []
        for c in text:
            if c in self._token_dict:
                R.append(c)
            else:
                R.append('[UNK]')   # The remaining characters are [UNK]
        return R


def save_to_file(file_name, contents):
    """
    save prediction label to file
    """
    fh = open(file_name, 'w')
    fh.write(contents)
    fh.close()


def preprocess(text):
    """
    tokenizer text to ID. Fill with 0 if the text  is not long than maxlen
    """
    tokenizer = OurTokenizer(token_dict)

    # tokenize
    text = text[:maxlen]
    x1, x2 = tokenizer.encode(first=text)

    X1 = x1 + [0] * (maxlen - len(x1)) if len(x1) < maxlen else x1
    X2 = x2 + [0] * (maxlen - len(x2)) if len(x2) < maxlen else x2
    return X1, X2


def postprocess(result_list):
    """
    get the category with max confidence
    """
    print(result_list[0]) 
    y = np.argmax(result_list[0])
    return y
    
    
def main():
    """
    acl resource initialization
    """
      
    acl_resource = AclResource()
    acl_resource.init()
    
    model = Model(model_path)

    with codecs.open(dict_path, 'r', 'utf-8') as reader:
        for line in reader:
            token = line.strip()
            token_dict[token] = len(token_dict) 

    with open(sample_path, "r") as f:
        text = f.read() 

    with open(label_path, "r", encoding="utf-8") as f:
        label_dict = json.loads(f.read())
               
    X1, X2 = preprocess(text)
    
    X1 = np.ascontiguousarray(X1, dtype='float32')
    X2 = np.ascontiguousarray(X2, dtype='float32')

    X1 = np.expand_dims(X1, 0)
    X2 = np.expand_dims(X2, 0)
    s_time = time.time()
    
    result_list = model.execute([X1, X2])
    e_time = time.time()    
    print(result_list)   
    y = postprocess(result_list)

    if not os.path.exists(output_dir):
        os.mkdir(output_dir)
    save_to_file(output_dir + 'prediction_label.txt', label_dict[str(y)])
    print("Original text: %s" % text)
    print("Prediction label: %s" % label_dict[str(y)])
    
    print("Cost time:", e_time - s_time)
    print("Execute end")

  
if __name__ == '__main__':
    main()

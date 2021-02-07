import os

def convertModel(input_width, input_height, model_type):
    input_width = str(input_width)
    input_height = str(input_height)
    command = ''' 
        atc \
            --model=model/{}.prototxt \
            --weight=model/{}.caffemodel \
            --framework=0 \
            --input_format=NCHW \
            --input_shape="data: 1, 1, {}, {}" \
            --output=model/{}_{}_{} \
            --soc_version=Ascend310 \
            --output_type=FP32 '''.format(model_type, 
                                          model_type, 
                                          input_height, input_width, 
                                          model_type, input_width, input_height)
    os.system(command)

if __name__ == "__main__":
    #######################################################
    # convertModel(input_width, input_height, model_type) #
    #######################################################

    convertModel(1536, 1536, 'SRCNN')
    convertModel(864,  864,  'SRCNN')
    convertModel(768,  768,  'SRCNN')
    convertModel(840,  840,  'SRCNN')
    convertModel(684,  1032, 'SRCNN')

    convertModel(512, 512, 'FSRCNN')
    convertModel(288, 288, 'FSRCNN')
    convertModel(256, 256, 'FSRCNN')
    convertModel(280, 280, 'FSRCNN')
    convertModel(228, 344, 'FSRCNN')

    convertModel(512, 512, 'ESPCN')
    convertModel(288, 288, 'ESPCN')
    convertModel(256, 256, 'ESPCN')
    convertModel(280, 280, 'ESPCN')
    convertModel(228, 344, 'ESPCN')

    convertModel(1536, 1536, 'VDSR')
    convertModel(864,  864,  'VDSR')
    convertModel(768,  768,  'VDSR')
    convertModel(840,  840,  'VDSR')
    convertModel(684,  1032, 'VDSR')

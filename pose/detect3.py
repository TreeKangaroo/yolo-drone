
#import tensorrt as trt 
import numpy as np
import onnx
import onnxruntime as ort

# load depth data as np array
input_file_path = 'data.npy'
depth_data = np.load(input_file_path)
depth_data = (depth_data/65536).astype(np.float32)

filename = "donModel.onnx"
input_node = 'input'

cnt = 0
while cnt<10:
    ort_sess = ort.InferenceSession(filename)
    outputs = ort_sess.run(None, {input_node: depth_data})
    print(cnt, outputs)
    cnt += 1

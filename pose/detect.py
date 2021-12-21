
import engine as eng
import inference as inf
import tensorrt as trt 
import numpy as np

TRT_LOGGER = trt.Logger(trt.Logger.WARNING)
trt_runtime = trt.Runtime(TRT_LOGGER)

# load depth data as np array
input_file_path = 'data.npy'
depth_data = np.load(input_file_path)
depth_data = depth_data/65536
print(depth_data.shape)

serialized_plan_fp32 = "donModel.plan"

engine = eng.load_engine(trt_runtime, serialized_plan_fp32)
h_input, d_input, h_output, d_output, stream = inf.allocate_buffers(engine, trt.float32)
cnt = 0
while cnt<10:
    out = inf.do_inference(engine, depth_data, h_input, d_input, h_output, d_output, stream)
    print(cnt, out)
    cnt +=1

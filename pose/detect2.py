import tensorrt as trt 
import pycuda.driver as cuda
import pycuda.autoinit 
import numpy as np
import cv2

class pose:
    # constructor
    def __init__(self):
        self.TRT_LOGGER = trt.Logger(trt.Logger.WARNING)
        self.trt_runtime = trt.Runtime(self.TRT_LOGGER)
        self.load_engine('donModel.plan')
        self.allocate_buffers(trt.float32)
        
    def load_engine(self, plan_path):
        with open(plan_path, 'rb') as f:
            engine_data = f.read()
        self.engine = self.trt_runtime.deserialize_cuda_engine(engine_data)
    
    def allocate_buffers(self, data_type):
        # Determine dimensions and create page-locked memory buffers (which won't be swapped to disk) to hold host inputs/outputs.
        self.h_input = cuda.pagelocked_empty(trt.volume(self.engine.get_binding_shape(0)), dtype=trt.nptype(data_type))
        self.h_output = cuda.pagelocked_empty(trt.volume(self.engine.get_binding_shape(1)), dtype=trt.nptype(data_type))
        # Allocate device memory for inputs and outputs.
        self.d_input = cuda.mem_alloc(self.h_input.nbytes)
        self.d_output = cuda.mem_alloc(self.h_output.nbytes)
        # Create a stream in which to copy inputs/outputs and run inference.
        self.stream = cuda.Stream()
            
    def do_inference(self, data):
        preprocessed = np.asarray(data).ravel()
        np.copyto(self.h_input, preprocessed) 

        with self.engine.create_execution_context() as context:
            # Transfer input data to the GPU.
            cuda.memcpy_htod_async(self.d_input, self.h_input, self.stream)

            # Run inference.
            context.profiler = trt.Profiler()
            context.execute(batch_size=1, bindings=[int(self.d_input), int(self.d_output)])

            # Transfer predictions back from the GPU.
            cuda.memcpy_dtoh_async(self.h_output, self.d_output, self.stream)
            # Synchronize the stream
            self.stream.synchronize()
            # Return the host output.
            out = self.h_output
            return out        
         
    def detection(self, depth_data):
        # perform pose detection
        depth_data = (depth_data/65536).astype(np.float32)
        out = self.do_inference(depth_data)
        print(out)
               

def main():
    pose_detect = pose()
    
    # load depth data as np array
    input_file_path = 'data.npy'
    depth_data = np.load(input_file_path)
    print(depth_data.shape)
    pose_detect.detection(depth_data)
    
if __name__ == '__main__':
    main()

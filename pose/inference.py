import tensorrt as trt
import pycuda.driver as cuda
import numpy as np
import pycuda.autoinit 

def allocate_buffers(engine, data_type):

   """
   This is the function to allocate buffers for input and output in the device
   Args:
      engine : The path to the TensorRT engine. 
      data_type: The type of the data for input and output, for example trt.float32. 
   
   Output:
      h_input_1: Input in the host.
      d_input_1: Input in the device. 
      h_output_1: Output in the host. 
      d_output_1: Output in the device. 
      stream: CUDA stream.

   """

   # Determine dimensions and create page-locked memory buffers (which won't be swapped to disk) to hold host inputs/outputs.
   h_input_1 = cuda.pagelocked_empty(trt.volume(engine.get_binding_shape(0)), dtype=trt.nptype(data_type))
   h_output = cuda.pagelocked_empty(trt.volume(engine.get_binding_shape(1)), dtype=trt.nptype(data_type))
   # Allocate device memory for inputs and outputs.
   d_input_1 = cuda.mem_alloc(h_input_1.nbytes)

   d_output = cuda.mem_alloc(h_output.nbytes)
   # Create a stream in which to copy inputs/outputs and run inference.
   stream = cuda.Stream()
   return h_input_1, d_input_1, h_output, d_output, stream 

def load_data_to_buffer(data, pagelocked_buffer):
   preprocessed = np.asarray(data).ravel()
   np.copyto(pagelocked_buffer, preprocessed) 

def do_inference(engine, data, h_input_1, d_input_1, h_output, d_output, stream):
   """
   This is the function to run the inference
   Args:
      engine : Path to the TensorRT engine 
      pics_1 : Input images to the model.  
      h_input_1: Input in the host         
      d_input_1: Input in the device 
      h_output_1: Output in the host 
      d_output_1: Output in the device 
      stream: CUDA stream
      height: Height of the output image
      width: Width of the output image
   
   Output:
      The list of output images

   """

   load_data_to_buffer(data, h_input_1)

   with engine.create_execution_context() as context:
       # Transfer input data to the GPU.
       cuda.memcpy_htod_async(d_input_1, h_input_1, stream)

       # Run inference.

       context.profiler = trt.Profiler()
       context.execute(batch_size=1, bindings=[int(d_input_1), int(d_output)])

       # Transfer predictions back from the GPU.
       cuda.memcpy_dtoh_async(h_output, d_output, stream)
       # Synchronize the stream
       stream.synchronize()
       # Return the host output.
       out = h_output       # may directpy return h_output
       return out 
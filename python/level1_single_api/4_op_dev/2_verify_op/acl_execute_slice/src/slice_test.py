import numpy as np
import acl


acl.init()
acl.rt.set_device(0)
context, ret = acl.rt.create_context(0)
stream, ret = acl.rt.create_stream()
op_attr_sqvar = acl.op.create_attr()
op_type_sqvar = 'Slice'
ret = acl.op.set_model_dir("../op_models_fp16")

inputs_desc_sqvar = []
inputs_device_buffer_sqvar = []
output_desc_sqvar = []
device_outputs_sqvar = []
device_buffer_outputs_sqvar = []

tensor_input_1 = acl.create_tensor_desc(1,[600,3000],2)
factor_size_600 = acl.get_tensor_desc_size(tensor_input_1)
inputs_desc_sqvar.append(tensor_input_1)
tensor_input_2 = acl.create_tensor_desc(3,[2],2)
factor_size_2 = acl.get_tensor_desc_size(tensor_input_2)
inputs_desc_sqvar.append(tensor_input_2)
tensor_input_3 = acl.create_tensor_desc(3,[2],2)
inputs_desc_sqvar.append(tensor_input_3)

tensor_output = acl.create_tensor_desc(1,[600,3000],2)
factor_device, ret = acl.rt.malloc(factor_size_600, 0)
device_outputs_sqvar.append(factor_device)
device_buffer_outputs_sqvar.append(acl.create_data_buffer(factor_device, factor_size_600))
output_desc_sqvar.append(tensor_output)

handle_sqvar, ret = acl.op.create_handle(op_type_sqvar, inputs_desc_sqvar, output_desc_sqvar, op_attr_sqvar)

in_matrix = np.ones((600,3000),dtype=np.float16)

bytes_in_matrix = in_matrix.tobytes()
in_matrix_ptr = acl.util.bytes_to_ptr(bytes_in_matrix)
in_matrix_buffer = acl.create_data_buffer(in_matrix_ptr, factor_size_600)
inputs_device_buffer_sqvar.append(in_matrix_buffer)

in_offsets = np.zeros(tuple([2]),dtype=np.int32)

bytes_in_offsets = in_offsets.tobytes()
in_offsets_ptr = acl.util.bytes_to_ptr(bytes_in_offsets)
in_offsets_buffer = acl.create_data_buffer(in_offsets_ptr, factor_size_2)
inputs_device_buffer_sqvar.append(in_offsets_buffer)

in_size = np.zeros(tuple([2]),dtype=np.int32)
in_size[0] = 600
in_size[1] = 3000

bytes_in_size = in_size.tobytes()
in_size_ptr = acl.util.bytes_to_ptr(bytes_in_size)
in_size_buffer = acl.create_data_buffer(in_size_ptr, factor_size_2)
inputs_device_buffer_sqvar.append(in_size_buffer)

ret = acl.op.execute_with_handle(handle_sqvar, inputs_device_buffer_sqvar, device_buffer_outputs_sqvar, stream)
print(ret)
ret = acl.rt.synchronize_stream(stream)
print(ret)

bytes_out = acl.util.ptr_to_bytes(device_outputs_sqvar[0], factor_size_600)
np_arr_out = np.frombuffer(bytes_out, dtype=np.float16).reshape((600,3000))

ua,uind=np.unique(np_arr_out,return_inverse=True)
count=np.bincount(uind)
print("count:",count)
acl.op.destroy_handle(handle_sqvar)
while inputs_desc_sqvar:
    ret = acl.destroy_data_buffer(inputs_device_buffer_sqvar.pop())
    acl.destroy_tensor_desc(inputs_desc_sqvar.pop())
while output_desc_sqvar:
    ret = acl.destroy_data_buffer(device_buffer_outputs_sqvar.pop())
    ret = acl.rt.free(device_outputs_sqvar.pop())
    acl.destroy_tensor_desc(output_desc_sqvar.pop())
acl.op.destroy_attr(op_attr_sqvar)
op_attr_sqvar = None

acl.rt.destroy_stream(stream)
acl.rt.destroy_context(context)
acl.rt.reset_device(0)
acl.finalize()

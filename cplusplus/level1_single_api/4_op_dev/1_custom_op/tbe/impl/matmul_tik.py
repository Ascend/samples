from te import tik
from te.tik.common.util import reduce_mul, ceil_div, DTYPE_SIZE
from te.platform.cce_conf import te_set_l2_mode
import numpy as np


def matmul_tik_compute(params, kernel_name):
    te_set_l2_mode(1)
    tik_instance = tik.Tik()
    if not isinstance(params, dict):
        params = params.__dict__
    m_size, k_size, n_size = params['M'], params['K'], params['N']
    data_type = params["data_type"]
    m_tiling_size = int(params["m_tiling_size"])
    n_tiling_size = int(params["n_tiling_size"])
    k_tiling_size = int(params['k_tiling_size'])

    m_cycle_times = params["m_cycle_times"]
    n_cycle_times = params["n_cycle_times"]
    k_cycle_times = params["k_cycle_times"]

    # Determine the output type
    if data_type == "float16":
        C_loc_out_type = "float32"
        K0 = 16
    else:
        C_loc_out_type = "int32"
        K0 = 32
    block_size = 16

    n_thread_num = params['n_thread_num']
    m_thread_num = params['m_thread_num']
    k_thread_num = params['k_thread_num']

    # Occupy the input tensor.
    output_gm = tik_instance.Tensor(C_loc_out_type, (n_size // block_size,
                                                     m_size, block_size),
                                    name="C_gm", scope=tik.scope_gm)
    inputa_gm = tik_instance.Tensor(params["data_type"], (k_size // K0,
                                                          m_size, K0),
                                    name="A_gm", scope=tik.scope_gm)
    inputb_gm = tik_instance.Tensor(params["data_type"], (k_size // K0,
                                                          n_size, K0),
                                    name="B_gm", scope=tik.scope_gm)

    # Tiling is realized through the for_range() loop.
    with tik_instance.for_range(0, 2, block_num = 2) as core_id:
        with tik_instance.for_range(0, n_cycle_times // 2,
                                    thread_num=n_thread_num) as n_idx:
            with tik_instance.for_range(0, m_cycle_times,
                                        thread_num=m_thread_num) as m_idx:
                dst_l0c = tik_instance.Tensor(C_loc_out_type,
                                              [n_tiling_size // 16,
                                               m_tiling_size, 16],
                                              name='dst_l0c',
                                              scope=tik.scope_cbuf_out)
                with tik_instance.for_range(0, k_cycle_times,
                                            thread_num=k_thread_num) as k_idx:
                    # Calculation result data transfer.
                    inputa_l1 = tik_instance.Tensor(params['data_type'],
                                               [k_tiling_size // K0,
                                                m_tiling_size, K0],
                                               name="A_tiling_l1",
                                               scope=tik.scope_cbuf)
                    tik_instance.data_move(inputa_l1,
                                           inputa_gm[k_idx *
                                                     k_tiling_size // K0,
                                           m_idx * m_tiling_size, :],
                                           0, k_tiling_size // K0, m_tiling_size,
                                           m_size - m_tiling_size, 0)
                    inputb_l1 = tik_instance.Tensor(params["data_type"],
                                               [k_tiling_size // K0,
                                                n_tiling_size, K0],
                                               name="B_tiling_l1",
                                               scope=tik.scope_cbuf)
                    if n_size - n_tiling_size > 65535:
                        with tik_instance.for_range(0, k_tiling_size // K0) \
                                as dma_k_idx:
                            tik_instance.data_move(inputb_l1[dma_k_idx, :, :],
                                                   inputb_gm[k_idx *
                                                             k_tiling_size //
                                                             K0 + dma_k_idx,
                                                   (core_id * n_cycle_times // 2 +
                                                    n_idx) * n_tiling_size, :],
                                                    0, 1, n_tiling_size, 0, 0)
                    else:
                        tik_instance.data_move(inputb_l1,
                                               inputb_gm[k_idx *
                                                         k_tiling_size // K0,
                                               (core_id * n_cycle_times // 2 +
                                                n_idx) * n_tiling_size, :], 0,
                                               k_tiling_size // K0,
                                               n_tiling_size,
                                               n_size - n_tiling_size, 0)
                    # Call matmul API to matrix multiplication calculation.
                    with tik_instance.if_scope(k_idx == 0):
                        tik_instance.matmul(dst_l0c, inputa_l1, inputb_l1,
                                            m_tiling_size,
                                            k_tiling_size, n_tiling_size,
                                            init_l1out=True)
                    with tik_instance.else_scope():
                        tik_instance.matmul(dst_l0c, inputa_l1, inputb_l1,
                                            m_tiling_size,
                                            k_tiling_size, n_tiling_size,
                                            init_l1out=False)
                tik_instance.fixpipe(output_gm[n_tiling_size // 16 * (core_id *
                                                                  n_cycle_times
                                                                  // 2 +
                                                                  n_idx),
                                     m_idx * m_tiling_size, :], dst_l0c,
                                     n_tiling_size // 16,
                                     m_tiling_size * 16 *
                                     DTYPE_SIZE[C_loc_out_type]//32,
                                     (m_size - m_tiling_size) * 16 *
                                     DTYPE_SIZE[C_loc_out_type] // 32, 0)

    tik_instance.BuildCCE(kernel_name=kernel_name,
                          inputs=[inputa_gm, inputb_gm], outputs=[output_gm])
    return tik_instance


def matmul_tik(input_x1, input_x2, output_y=None, kernel_name="simple_matmul"):
    """
    matmul_tik main func
    Parameters
    ----------
    input_x1: input data 1
    input_x2: input data 2
    output_y: output dta
    """
    shape_a = input_x1.get("ori_shape")
    shape_b = input_x2.get("ori_shape")
    output_y = output_y
    m = shape_a[0]
    k = shape_a[1]
    n = shape_b[1]
    data_type = input_x1.get("dtype").lower()
    params = {
        'M': m,
        'K': k,
        'N': n,
        'data_type': data_type,
        'm_tiling_size': 16,
        'm_cycle_times': 1,
        'm_thread_num': 1,
        'n_tiling_size': 64,
        'n_cycle_times': 16,
        'n_thread_num': 1,
        'k_tiling_size': 32,
        'k_cycle_times': 2,
        'k_thread_num': 2,
        'output_y':output_y
    }
    return matmul_tik_compute(params, kernel_name)

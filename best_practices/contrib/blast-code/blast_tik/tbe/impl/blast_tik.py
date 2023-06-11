import math
from functools import reduce as functools_reduce
from tbe import tik
import te.platform as tbe_platform
from tbe.common.utils import para_check
import numpy as np

GAP = -5
class Blast():

    def __init__(self, seq1, seq2, scores, out, kernel_name):
        """
    Init blast base parameters

    Parameters
    ------------
    seq1: dict
        data of input
        datatype supports int32,int16
    seq2: dict
        data of input
        datatype supports int32,int16
    scores: dict
        data of temp value
        datatype supports int32,int16
    out: dict
        data of output
    kernel_name: str
            the name of the operator
    Returns
    -------
    None
    """
        self.tik_instance = tik.Tik(disable_debug=False)
        self.seq1_shape = seq1.get("shape")
        self.seq1_dtype = seq1.get("dtype").lower()
        self.seq2_shape = seq2.get("shape")
        self.seq2_dtype = seq2.get("dtype").lower()
        self.out_shape = out.get("shape")
        self.out_dtype = out.get("dtype").lower()
        self.scores_shape = scores.get("shape")
        self.scores_dtype = scores.get("dtype").lower()
        self.seq1_dna_num = self.seq1_shape[0]
        self.seq2_dna_num = functools_reduce(lambda x, y: x * y,
                                             self.seq2_shape)
        self.scores_num = self.scores_shape[0]
        self.out_num = self.out_shape[0]
        self.kernel_name = kernel_name
        self.check_param()
        # 构造TIK容器并开启debug调试功能
        # 获取AI core的个数
        self.aicore_num = tbe_platform.get_soc_spec("CORE_NUM")
        # 获取Unified Buffer空间大小，单位为Bytes
        self.ub_size_bytes = tbe_platform.get_soc_spec("UB_SIZE")
        # 根据输入的数据类型计算一个block可以存放多少个对应的元素
        self.seq1_dtype_bytes_size = tbe_platform.get_bit_len(self.seq1_dtype) // 8
        self.seq2_dtype_bytes_size = tbe_platform.get_bit_len(self.seq2_dtype) // 8
        self.scores_dtype_bytes_size = tbe_platform.get_bit_len(self.scores_dtype) // 8
        self.out_dtype_bytes_size = tbe_platform.get_bit_len(self.out_dtype) // 8
        # UB的数据读写必须32B对齐
        self.seq1_data_each_block = 32 // self.seq1_dtype_bytes_size
        self.seq2_data_each_block = 32 // self.seq2_dtype_bytes_size
        self.scores_data_each_block = 32 // self.scores_dtype_bytes_size
        self.out_data_each_block = 32 // self.out_dtype_bytes_size
        self.seq1_ub_number = 0
        self.seq2_ub_number = 0

        # 为输入、输出张量在内存中申请空间
        self.seq1_gm = self.tik_instance.Tensor(self.seq1_dtype, self.seq1_shape, name="seq1_gm",
                                                scope=tik.scope_gm)
        self.seq2_gm = self.tik_instance.Tensor(self.seq2_dtype, self.seq2_shape, name="seq2_gm",
                                                scope=tik.scope_gm)
        self.scores_gm = self.tik_instance.Tensor(self.scores_dtype, self.scores_shape,
                                                  name="scores_gm",
                                                  scope=tik.scope_gm)
        self.out_gm = self.tik_instance.Tensor(self.out_dtype, self.out_shape, name="out_gm",
                                               scope=tik.scope_gm)


        # 计算每个AI Core需要处理的数据量，因为AI Core数量小于seq2[1]，所以均分seq2[1]
        # 每个AI Core均分seq2，再加一个seq1和scores，且分后32 Bytes对齐
        self.each_core_seq2_dim0 = self.seq2_shape[0] // self.aicore_num
        self.data_num_each_core = self.seq2_dna_num // self.aicore_num

        self.scores_M = (self.seq1_shape[0]+1)*(self.seq1_shape[0]+1)-1
        self.scores_dim = self.seq1_shape[0]+1

    def data_num_each_core(self, index):
        # 如果seq2的行数与aicore_num除不尽，根据结果分配每个核心的数据量
        seq2_num_each_core = self.seq2_num_each_core(index)
        return seq2_num_each_core + self.seq1_dna_num + self.scores_num

    # 每个core中seq2的元素数量
    def seq2_num_each_core(self, index):
        each_core_seq2_row = self.seq2_row_each_core(index)
        return each_core_seq2_row * self.seq2_shape[1]

        
    def blast_compute(self):
        with self.tik_instance.for_range(0, self.aicore_num,block_num=self.aicore_num) as index:

            # 创建在Unified Buffer上的tensor，seq1的shape是它本身
            self.input_seq1_ub = self.tik_instance.Tensor(self.seq1_dtype, self.seq1_shape, name="input_seq1_ub",
                                                          scope=tik.scope_ubuf)
            self.input_seq2_ub = self.tik_instance.Tensor(self.seq2_dtype,
                                                          (self.each_core_seq2_dim0, self.seq2_shape[1]),
                                                          name="input_seq2_ub",
                                                          scope=tik.scope_ubuf)
            self.indice_ub = self.tik_instance.Tensor(self.seq2_dtype,(self.seq1_shape[0],),
                                                      name="indice_ub",
                                                        scope=tik.scope_ubuf)
            self.scores_ub = self.tik_instance.Tensor(self.scores_dtype, self.scores_shape, name="scores_ub",
                                                      scope=tik.scope_ubuf)
            self.out_ub = self.tik_instance.Tensor(self.out_dtype, (self.each_core_seq2_dim0,), name="out_ub",
                                                   scope=tik.scope_ubuf)
            # 将对应的GM上的数据搬运到Unified Buffer，每次搬运的偏移量为已经处理过的数据个数
            self.each_core_seq2_row = self.tik_instance.Scalar("int16")
            self.each_core_seq2_row.set_as(0)
            move_offset = index * self.data_num_each_core
            self.each_core_seq2_row.set_as(self.each_core_seq2_dim0)
            self.row_offset = self.tik_instance.Scalar("int16")
            self.row_offset.set_as(index * self.each_core_seq2_dim0)
            # 每个aicore计算自己负责的数据分片
            self.blast_compute_each_core(move_offset)
        self.tik_instance.BuildCCE(
            kernel_name=self.kernel_name,
            inputs=[self.seq1_gm, self.seq2_gm, self.scores_gm],
            outputs=[self.out_gm]
        )
        return self.tik_instance

    def scores_init(self):
        scores_length = self.seq1_shape[0]
        self.tik_instance.vec_dup(100, self.scores_ub, 0, 1, 0)
        with self.tik_instance.for_range(0, scores_length+1) as i:
            with self.tik_instance.if_scope(i > 0):
                self.scores_ub[i*self.scores_dim].set_as(GAP*i)
            with self.tik_instance.else_scope():
                with self.tik_instance.for_range(1, scores_length+1) as j:
                    self.scores_ub[j].set_as(GAP*j)

    def blast_compute_each_core(self, move_offset):
        seq1_burst_len = math.ceil(self.seq1_dna_num / self.seq1_data_each_block)
        seq2_burst_len = math.ceil(self.data_num_each_core / self.seq2_data_each_block)
        scores_burst_len = math.ceil(self.scores_num / self.scores_data_each_block)
        out_burst_len = math.ceil(self.each_core_seq2_dim0 / self.out_data_each_block)
        # seq1 move to ub
        self.tik_instance.data_move(self.input_seq1_ub,
                                    self.seq1_gm,0,1,
                                    seq1_burst_len,0,0)
        # seq2需计算偏移
        self.tik_instance.data_move(self.input_seq2_ub,
                                    self.seq2_gm[move_offset],0,1,
                                    seq2_burst_len,0,0)
        # scores全部搬进ub
        self.tik_instance.data_move(self.scores_ub,
                                    self.scores_gm,0,1,
                                    scores_burst_len,0,0)
        self.tik_instance.data_move(self.out_ub,
                                    self.out_gm,0,1,
                                    out_burst_len,0,0)
        self.diagonal_score = self.tik_instance.Scalar(dtype="int16", name="diagonal_score")
        self.left_score = self.tik_instance.Scalar(dtype="int16", name="left_score")
        self.up_score = self.tik_instance.Scalar(dtype="int16", name="up_score")
        self.diag_tmp = self.tik_instance.Scalar("int16")
        self.diag_tmp.set_as(0)
        self.left_tmp = self.tik_instance.Scalar("int16")
        self.left_tmp.set_as(0)
        self.up_tmp = self.tik_instance.Scalar("int16")
        self.up_tmp.set_as(0)
        self.col = self.tik_instance.Scalar("int16")
        self.col.set_as(0)
        self.out_tmp = self.tik_instance.Scalar("int16")
        self.out_tmp.set_as(0)
        self.scores_tmp = self.tik_instance.Scalar("int16")
        self.scores_tmp.set_as(self.seq1_shape[0])
        self.letter2 = self.tik_instance.Scalar(dtype="int16", name="letter2")
        self.letter1 = self.tik_instance.Scalar(dtype="int16", name="letter1")
        self.score_scalar = self.tik_instance.Scalar(init_value=0, dtype="int16", name="score_scalar")
        self.scores_init()

        #再加一层循环
        #偏移行数
        with self.tik_instance.for_range(0, self.each_core_seq2_row) as k:
            self.indice_ub = self.input_seq2_ub[k*self.seq1_shape[0]:(k+1)*self.seq1_shape[0]]
            with self.tik_instance.for_range(1, self.seq1_shape[0]+1) as index1:
                self.letter2.set_as(self.indice_ub[index1-1])
                with self.tik_instance.for_range(1, self.seq1_shape[0]+1) as index2:
                    self.score_scalar.set_as(0)
                    self.letter1.set_as(self.input_seq1_ub[index2-1])
                    with self.tik_instance.if_scope(self.letter1 == self.letter2):
                        self.score_scalar.set_as(self.score_scalar + 2)
                    with self.tik_instance.elif_scope(tik.any(self.letter1 * 10 + self.letter2 == 12, self.letter1 * 10 + self.letter2 == 34,
                                                              self.letter1 * 10 + self.letter2 == 21, self.letter1 * 10 + self.letter2 == 43)):
                        self.score_scalar.set_as(self.score_scalar - 5)
                    with self.tik_instance.else_scope():
                        self.score_scalar.set_as(self.score_scalar - 7)
                    self.diag_tmp.set_as(self.scores_ub[self.scores_dim*(index1-1)+index2-1])
                    self.diagonal_score.set_as(self.score_scalar + self.diag_tmp)
                    self.left_tmp.set_as(self.scores_ub[self.scores_dim*index1+index2-1])
                    self.left_score.set_as(GAP + self.left_tmp)
                    self.up_tmp.set_as(self.scores_ub[self.scores_dim*(index1-1)+index2])
                    self.up_score.set_as(GAP + self.up_tmp)
                    with self.tik_instance.if_scope(self.diagonal_score >= self.left_score):
                        with self.tik_instance.if_scope(self.diagonal_score >= self.up_score):
                            self.scores_ub[self.scores_dim*index1+index2].set_as(self.diagonal_score)
                        with self.tik_instance.else_scope():
                            self.scores_ub[self.scores_dim*index1+index2].set_as(self.up_score)
                    with self.tik_instance.else_scope():
                        with self.tik_instance.if_scope(self.left_score >= self.up_score):
                            self.scores_ub[self.scores_dim*index1+index2].set_as(self.left_score)
                        with self.tik_instance.else_scope():
                            self.scores_ub[self.scores_dim*index1+index2].set_as(self.up_score)
            self.out_tmp = self.scores_ub[self.scores_M]
            self.out_ub[k] = self.out_tmp
            self.scores_init()
        self.tik_instance.data_move(self.out_gm[self.row_offset], self.out_ub, 0, 1, out_burst_len, 0, 0)
        self.tik_instance.data_move(self.seq1_gm,
                                    self.input_seq1_ub,0,1,
                                    seq1_burst_len,0,0)
        self.tik_instance.data_move(self.seq2_gm[move_offset],
                                    self.input_seq2_ub,0,1,
                                    seq2_burst_len,0,0)
        # scores全部搬进ub
        self.tik_instance.data_move(self.scores_gm,
                                    self.scores_ub,0,1,
                                    scores_burst_len,0,0)


    def check_param(self):
        para_check.check_kernel_name(self.kernel_name)
        para_check.check_shape_rule(self.seq1_shape)
        para_check.check_shape_rule(self.seq2_shape)
        para_check.check_shape_rule(self.scores_shape)
        para_check.check_shape_rule(self.out_shape)

        para_check.check_shape_size(self.seq1_shape)
        para_check.check_shape_size(self.seq2_shape)
        para_check.check_shape_size(self.scores_shape)
        para_check.check_shape_size(self.out_shape)

        check_list = ("int16", "float16")
        para_check.check_dtype_rule(self.seq1_dtype, check_list)
        para_check.check_dtype_rule(self.seq2_dtype, check_list)
        para_check.check_dtype_rule(self.scores_dtype, check_list)
        para_check.check_dtype_rule(self.out_dtype, check_list)

        if self.seq1_dtype != self.seq2_dtype:
            raise RuntimeError(
                "All params' datatype must be the same"
            )
        if self.seq2_shape[0] != self.out_shape[0]:
            raise RuntimeError(
                "seq1's shape must be the same as out's shape"
            )
        if (self.scores_shape[0] < (self.seq1_shape[0] * self.seq1_shape[0])):
            raise RuntimeError(
                "scores's shape too small to run the op"
            )


def blast_tik(seq1, seq2, scores, out, kernel_name="blast_tik"):
    blast = Blast(seq1, seq2, scores, out, kernel_name)

    tik_instance = blast.blast_compute()

    return tik_instance

if __name__ == "__main__":
    tik_instance = blast_tik({"shape":(16, ), "dtype":"int16"},
                               {"shape":(32, 16), "dtype":"int16"},
                               {"shape":(400,), "dtype":"int16"},
                                {"shape":(32, ), "dtype":"int16"},
                               "blast_tik")

    seq1 = np.random.randint(1, 5,(16,)).astype("int16")
    seq2 = np.random.randint(1, 5,(32, 16)).astype("int16")
    scores = np.zeros(shape=(400,)).astype("int16")
    out = np.zeros(shape=(32)).astype("int16")
    feed_dict = {
        'seq1_gm': seq1,
        'seq2_gm': seq2,
        'scores_gm': scores
    }
    result = tik_instance.tikdb.start_debug(feed_dict, interactive=True)
    print(result)
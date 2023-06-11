import numpy as np


def get_exp_data(seq1,seq2,scores,out):

    s1 = seq1["value"].astype("int16")
    print(s1)
    s2 = seq2["value"].astype("int16")
    score = np.zeros(shape=(s2.shape[0],s1.shape[0]+1,s1.shape[0]+1), dtype=int).astype("int16")
    res = np.zeros(shape=(s2.shape[0]),dtype=int).astype("int16")
    m = s1.shape[0]
    n = s2.shape[1]
    GAP = -5
    for k in range(s2.shape[0]):
        sp = s2[k]
        for j in range(n + 1):
            if j == 0:
                for i in range(n + 1):
                    score[k, j, i] = GAP * i
            else:
                score[k, j, 0] = GAP * j

        for j in range(1, n + 1):
            letter2 = sp[j - 1]
            for i in range(1, m + 1):
                letter1 = s1[i - 1]
                sc = 0
                if letter1 == letter2:
                    sc += 2
                elif letter1 * 10 + letter2 == 12 or letter1 * 10 + letter2 == 34 or letter1 * 10 + letter2 == 21 or letter1 * 10 + letter2 == 43:
                    sc += -5
                else:
                    sc += -7
                diagonal_score = sc + score[k, j - 1, i - 1]
                left_score = GAP + score[k, j, i - 1]
                up_score = GAP + score[k, j - 1, i]
                max_score = max(diagonal_score, left_score, up_score)
                score[k, j, i] = max_score

        res[k] = score[k, n, m]
    return [res]
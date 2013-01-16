import numpy as np

def graycode_str(n):
    assert n>=1
    if n==1:
        return ['0','1']
    else:
        prev = graycode_str(n-1)
        reflect = prev[::-1]

        prev = [ '0'+i for i in prev ]
        reflect = ['1'+i for i in reflect ]
        concat = prev + reflect
        return concat

def graycode_arange(maxval, dtype=np.uint16):
    maxbits = int(np.ceil(np.log2( maxval )))
    gs = graycode_str(maxbits)
    gs = [ int(g,2) for g in gs ]
    result = np.array(gs,dtype=dtype)
    return result[:maxval]

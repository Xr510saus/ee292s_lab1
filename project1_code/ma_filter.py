#!/usr/bin/python
# -*- coding:utf-8 -*-

import numpy as np

def maf(signal: list, z: float, fil_len: int=3) -> float:
    '''
    Inputs:
        signal: list of floats # Should be at least fil_len-1 samples long
        z: float # Current measurement
        fil_len: int # length of the moving average filter
    Outputs:
        output: float # Output of the current moving average
    '''
    
    block = signal[-(fil_len-1):]
    
    output = (np.sum(block) + z) / len(block)
    
    return output
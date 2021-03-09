# -*- coding: utf-8 -*-
"""
Created on Wed Jul  8 13:02:05 2020

@author: azheng
"""


class Test(object):
    def __init__(self, input):
        self.a_ = 0
        self.b_ = 1
        self.input = input
        
    def runTest(self):
        print("Run Test")

if __name__ == '__main__':

    T = Test(1.0) 
    
    #print("T input is " + str(T.input))
    
    T.runTest()
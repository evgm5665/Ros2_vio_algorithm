import numpy as np 
import pandas as pa
import pathlib as Path

#script converts the log files to csv files for easier analysis

GTLogPath = "/home/usl/Ovio_ws/src/vio_alg/Bag_log/GroundTruthLogs/"
OVLogPath = "/home/usl/Ovio_ws/src/vio_alg/Bag_log/OpenVinsEstLogs/"


def input_GTLogPath():
    
    print("Input name of Ground Truth Log file:")
    GL = input()
    return GL

def input_OVLogPath():
    
    print("Input name of OpenVins Log file: ")
    OL = input()
    return OL

#TO-DO: make function to group the original paths with the inputted file names to 
# create the full path to the log files

if __name__ == "__main__":
   
    GLogName = input_GTLogPath()
    OLogName = input_OVLogPath()


    print("Ground Truth Log File: ", f"{GTLogPath}{GLogName}")
    print("OpenVins Log File: ", f"{OVLogPath}{OLogName}")

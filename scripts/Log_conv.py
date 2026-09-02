import numpy 
import pandas as pa
import pathlib as Path
import csv
import re

#script converts the log files to csv files for easier analysis

GTLogPath = "/home/usl/Ovio_ws/src/vio_alg/Bag_log/GroundTruthLogs/"

OVLogPath = "/home/usl/Ovio_ws/src/vio_alg/Bag_log/OpenVinsEstLogs/"



#paths to txt file
def input_GTLogPath():
    
    print("Input name of Ground Truth Log file:")
    GL = input()
    return GL
def input_OVLogPath():
    
    print("Input name of OpenVins Log file: ")
    OL = input()
    return OL
GLogName = input_GTLogPath()
OLogName = input_OVLogPath()
def GT_FinalPath(GLogName):
    Gname = f"{GTLogPath}{GLogName}.txt"
    return Gname

def OV_FinalPath(OLogName):
    Oname = f"{OVLogPath}{OLogName}.txt"
    return Oname
   

#final Paths to txt files
GT_input = GT_FinalPath(GLogName)
OV_input = OV_FinalPath(OLogName)

def OV_Conv(OV_input):
    #time(s), px, py, pz, qx, qy, qz, qw

    #pattern for initial time
    initTime = "successfull initialization in 0.000 seconds"
    IT_pattern = r"(?p<initTime>\d+)"

    FirstPosition = ""



if __name__ == "__main__":


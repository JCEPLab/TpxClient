from client import *
from server import TpxServer

MASK_CONFIG_FILE = '../config/maskBits.txt'
THRESHOLD_CONFIG_FILE = '../config/thlAdj.txt'
TEST_CONFIG_FILE = '../config/testBits.txt'
DAC_CONFIG_FILE = '../config/dacConfig.json'

if __name__ == '__main__':

    server = TpxServer(r"C:\Users\JCEP Upsilon\Documents\Kyle\TpxServer\build\Debug_msvc2022_x86-64\TpxServer.exe")
    server.start(path_append=r"C:\Qt\6.7.2\msvc2019_64\bin")

    client = TpxClient()
    client.initialize(MASK_CONFIG_FILE, THRESHOLD_CONFIG_FILE, TEST_CONFIG_FILE, DAC_CONFIG_FILE)

    client.takeSingleImage(10, "C:/Users/JCEP Upsilon/Desktop/output.tpx3")

    server.stop()

    #client.takeSingleImage(1, "C:\\Users\\JCEP Chi\\Desktop\\output1.tpx3")
    #client.takeSingleImage(1, "C:\\Users\\JCEP Chi\\Desktop\\output2.tpx3")
    #client.takeSingleImage(1, "C:\\Users\\JCEP Chi\\Desktop\\output3.tpx3")

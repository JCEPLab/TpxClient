from client import *
from raw_data_plotter import *

MASK_CONFIG_FILE = './config/maskBits.txt'
THRESHOLD_CONFIG_FILE = './config/thlAdj.txt'
TEST_CONFIG_FILE = './config/testBits.txt'
DAC_CONFIG_FILE = './config/dacConfig.json'

if __name__ == '__main__':

    client = TpxClient()
    client.initialize(MASK_CONFIG_FILE, THRESHOLD_CONFIG_FILE, TEST_CONFIG_FILE, DAC_CONFIG_FILE)

    client.takeSingleImage(10, "C:/Users/JCEP Upsilon/Desktop/output.tpx3")

    #client.takeSingleImage(1, "C:\\Users\\JCEP Chi\\Desktop\\output1.tpx3")
    #client.takeSingleImage(1, "C:\\Users\\JCEP Chi\\Desktop\\output2.tpx3")
    #client.takeSingleImage(1, "C:\\Users\\JCEP Chi\\Desktop\\output3.tpx3")

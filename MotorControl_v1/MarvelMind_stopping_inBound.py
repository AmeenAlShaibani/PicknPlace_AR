from MarvelMind import MarvelmindHedge
from time import sleep
import sys

def main():
    hedge = MarvelmindHedge(tty = "/dev/ttyACM1_MarvelMind", adr=None, debug=False) # create MarvelmindHedge thread
    

    if (len(sys.argv)>1):
        hedge.tty= sys.argv[1]
    
    hedge.start() # start thread
    while True:
        try:
            hedge.dataEvent.wait(1)
            hedge.dataEvent.clear()

            if (hedge.positionUpdated):
                hedge.print_position()
                if(hedge.position()[1] > 6.05
                   and hedge.position()[1] < 6.255
                   and hedge.position()[2] > 4.495
                   and hedge.position()[2] < 4.63):
                    print("Within Box")
                    hedge.stop()
                    sys.exit()
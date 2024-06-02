
# replaces px_uploader.py, fixing some problems arising form uploading within WSL

# import original code
from px_uploader import *

if __name__ == '__main__':

    port_bootloader  = "/dev/ttyS4,/dev/ttyS5,/dev/ttyS6,/dev/ttyS7"

    # remove __determineInterface check
    def _determineInterface(self):
        return
    uploader._uploader__determineInterface = _determineInterface
    
    # set arguments
    if len(sys.argv) != 2: print("bad syntax"); exit()
    sys.argv[1] = "build/"+sys.argv[1]+"/"+sys.argv[1]+".px4"
    sys.argv.append("--port")
    sys.argv.append(port_bootloader)
    
    # run original code
    main()
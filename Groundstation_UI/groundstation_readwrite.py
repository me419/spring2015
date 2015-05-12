import serial
import time

class GS_readwrite():
    """
    """
    def __init__(self,COMPORT,BAUDRATE,TIMEOUT,AUTOCONNECT = False):
        if AUTOCONNECT:
            self.autoconnect(BAUDRATE,TIMEOUT)
        else:
            self.conn = serial.Serial('COM%i'%COMPORT, BAUDRATE, timeout= TIMEOUT)
        if not self.conn.closed:
            print 'Successfully connected to Serial Device!'
        self.buffer = ""
        self.data   = {}
        
        self.STARTTIME = time.time()
        self.logfile = open('logfile.txt','a')
        
    def autoconnect(self,BAUDRATE,TIMEOUT):
        """Attempt to automatically connect to Serial Port."""
        import win32com.client
        wmi = win32com.client.GetObject("winmgmts:")
        for pnpEntity in wmi.InstancesOf("Win32_PnPEntity"):
            try:
                if 'FTDIBUS\\COMPORT&VID_0403' in pnpEntity.HardwareID[0]:
                    comport = pnpEntity.Name.split(' ')[-1][1:-1]
                    print comport
                    self.conn = serial.Serial(comport, BAUDRATE, timeout= TIMEOUT)
            except:pass   

    def read_receive(self):
        """
        Print received packet and save to buffer. If packet is not available
        return False.
        """
        try:
            readline = self.conn.readline()
            if type(readline) == None or readline == '':
                return False
            print readline
            self.buffer += readline
            return True
        except Exception as e:
            print "Data could not be read because of the following error:"
            print e
            return False
        

    def proc_buffer(self):
        """
        Process the contents of the buffer according to the following:
        <  : Begin Packet
        >  : Close Packet
        $X : Signifies the start of datatype X
        I  : Internal Temperature
        E  : External Temperature
        P  : Air Pressure
        L  : Latitude, Longitude datatype
        A  : Altitude
        V  : Velocity
        T  : (Roll,Pitch,Yaw) datatype
        R  : (Roll rate, Pitch rate, Yaw rate)
        V  : Arduino Voltage
        B  : Motor Voltage
        Z  : Autopilot Mode
        
        
        If a full packet is recieved the data is saved to the data dictionary
        with a timestamp as the key.
        """
        tempdata = ['' for _ in range(11)]
        typedict = dict(zip(['I','E','P','L','A','V','T','R','V','B','Z'],range(11)))
        def readpacket(p):
            parts = p.split('$')[1:]
            for part in parts:
                tempdata[typedict[part[0]]] = part[1:]
            return None
        packetcount = (self.buffer.count('<')+self.buffer.count('>'))/2
        bufferlist = self.buffer.split('<')
        packets = [s.replace('>','') for s in bufferlist[1:packetcount+1]]
        timestamp = time.strftime('%Y/%m/%d %H:%M:%S')
        for ind,packet in enumerate(packets):
            timestamp = timestamp[:-2]+str(int(timestamp[-2:])+ind)
            self.data[timestamp] = readpacket(packet)
            

    def send_packet(**kwargs):
        """
        Sends a packet
        for key, value in kwargs.iteritems():

        """
        pass
    
    

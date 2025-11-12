"""
Data Logger for AKP2 Project
Daniel Gonzalez
dgonz@mit.edu
"""

import os
import numpy


class dataLogger():
    # Instance buffer of rows
    myData = []

    def __init__(self, name):
        self.name = name
        # Ensure directory exists
        d = os.path.dirname(self.name)
        if d:
            os.makedirs(d, exist_ok=True)
        # Create/truncate file
        with open(self.name, 'w'):
            pass
    
    def writeOut(self):
        print('Storing data...\n')
        outTxt = []
        for data in self.myData:
            outTxt.append(' '.join(str(e) for e in data))
            outTxt.append('\n')
        if outTxt:
            with open(self.name, 'a') as f:
                f.write(''.join(outTxt))
        self.myData = []
        # numpy.savetxt(name,self.myData)

    def appendData(self,val):
        self.myData.append(val)
#!/usr/bin/env python

import time
import math
import pyaudio
import numpy as np
from scipy.signal import butter, lfilter, freqz
import matplotlib.pyplot as plt
import RPi.GPIO as GPIO
import spidev


### Class to support pyAudio ###
#class for writing to speaker
class Output:
    pa = None;
    s  = None;

    #call this function to open output and create stream
    def __init__(self,rate=8000):
        """
        Output class constructor initializes and opens pyAudio output
        stream. Allows sending audio data to an audio output device via python.
        """
        #print ("init_audio: Create PyAudio object")
        self.pa = pyaudio.PyAudio()
        #print ("init_audio: Open stream")
        self.s = self.pa.open(output=True,
                channels=1,
                rate=rate,
                format=pyaudio.paInt16,
                output_device_index=0)
        print ("audio stream initialized")

    def close_audio(self):
        """
        Output class destructor closes and destructs output object
        and associated pyAudio stream.
        """
        #print ("close_audio: Closing stream")
        self.s.close()
        print ("close_audio: Terminating PyAudio Object")
        self.pa.terminate()
        
    #create a numpy array of a given freqeuncy    
    def customNote(self, freq, length, amp=5000, rate=8000, 
            vibrato=0,vibratoFreq=0,
            trembelo=0, trembeloFreq=0):
        """
        Creates a uniform note (a sinusoidal waveform of with characteristics specified
        by function parameters) returned as a 16bit integer numpy array.
        """
      # get time array
        t = np.linspace(0,length,length*rate)
        # create a modulated tone
        data = np.sin(2*np.pi*(freq + vibrato*np.sin(2*np.pi*t*vibratoFreq))*t)               *(amp*trembelo*np.sin(2*np.pi*t*trembeloFreq))
        return data.astype(np.int16) # two byte integers

    # overloaded note function that derives note params from list of pot readings
    def note(self, freq, pot_readings):
        """
        Creates a uniform note (a sinusoidal waveform of with characteristics specified
        by function parameters) returned as a 16bit integer numpy array.
        """
        freq = 250
        len=15
        RATE = 8000
        amp = pot_readings[0]*4000 + 3000 # [3000, 7000]
        vibrato = pot_readings[1] # [0, 1]
        vibratoFreq = pot_readings[2]*100 # [0, 100]
        
        trembelo = pot_readings[3]*4000 # [0, 4000]
        #trembelo = 1
        trembeloFreq = pot_readings[4]*100 # [0, 100]
        print(freq, amp, vibrato, vibratoFreq, trembelo, trembeloFreq)
        # get time array
        t = np.linspace(0,len,len*RATE)
        # create a modulated note
        data = np.sin(2*np.pi*(freq + vibrato*np.sin(2*np.pi*t*vibratoFreq))*t)               *(amp+(trembelo*np.cos(2*np.pi*t*trembeloFreq)))
        
        return data.astype(np.int16) # two byte integers
    
    #write numpy array to stream
    def tone(self, note):
        """
        Takes a 16bit integer numpy array, representing a note, as an argument
        and plays the represented note via the pyAudio stream.
        """
        # generate sound
        self.s.write(note)
    
    #add two numpy arrays together
    def add_note(self, note1, note2):
        """
        combines two notes (of the same length) element-wise to produce a new note.
        """
        return note1 + note2


### Helper function library ###
def increaseFreq(ev=None):
    '''
    increments the global frequency value, if the value if not at maximum
    '''
    MAX_FREQ = 523.25
    global freq
    global keys_list
    if freq < MAX_FREQ:
        freq_idx = keys_list.index(freq)
        freq = keys_list[freq_idx+1]
    print('increase')
        
def decreaseFreq(ev=None):
    '''
    decrements the global frequency value, if the value if not at minimum
    '''
    MIN_FREQ = 261.63
    global freq
    global keys_list
    if freq > MIN_FREQ:
        freq_idx = keys_list.index(freq)
        freq = keys_list[freq_idx-1]
    print('decrease')

def initSpi():
    spi1 = spidev.SpiDev()       # Create a new spidev object
    spi1.open(0,0)
    spi1.max_speed_hz = int(20e5) # Set clock speed to 

    spi2 = spidev.SpiDev()       # Create a new spidev object
    spi2.open(0,1)
    spi2.max_speed_hz = int(20e5) # Set clock speed to ___
    
    GPIO.output(ADC2, True)
    GPIO.output(ADC3, True)
    
    return spi1, spi2

def readPiezo(spi1):
    config = [0b01101000, 0] # Measure from channel 0
    myBytes = spi1.xfer2(config) # Send and get array of 2 bytes from ADC
    myData = (myBytes[0] << 8) | myBytes[1] # Convert returned bytes to integer value
    return myData

def readPot(input, spi1, spi2):
    if(input==0):
        config = [0b01111000, 0] # Measure from channel 1
        myBytes = spi1.xfer2(config)             # Send and get array of 2 bytes from ADC
        myData = (myBytes[0] << 8) | myBytes[1] # Convert returned bytes to integer value
        return myData   
     
    if(input==1):
        config = [0b01101000, 0] # Measure from channel 0
        GPIO.output(ADC2, False)
        myBytes = spi2.xfer2(config)             # Send and get array of 2 bytes from ADC
        GPIO.output(ADC2, True)
        myData = (myBytes[0] << 8) | myBytes[1] # Convert returned bytes to integer value
        return myData
    
    if(input==2):
        config = [0b01111000, 0] # Measure from channel 1
        GPIO.output(ADC2, False)
        myBytes = spi2.xfer2(config)             # Send and get array of 2 bytes from ADC
        GPIO.output(ADC2, True)
        myData = (myBytes[0] << 8) | myBytes[1] # Convert returned bytes to integer value
        return myData
    
    if(input==3):
        config = [0b01101000, 0] # Measure from channel 0
        GPIO.output(ADC3, False)
        myBytes = spi2.xfer2(config)             # Send and get array of 2 bytes from ADC
        GPIO.output(ADC3, True)
        myData = (myBytes[0] << 8) | myBytes[1] # Convert returned bytes to integer value
        return myData
    
    if(input==4):
        config = [0b01111000, 0] # Measure from channel 1
        GPIO.output(ADC3, False)
        myBytes = spi2.xfer2(config)             # Send and get array of 2 bytes from ADC
        GPIO.output(ADC3, True)
        myData = (myBytes[0] << 8) | myBytes[1] # Convert returned bytes to integer value
        return myData

def normalizePotVal(val):
    '''
    Takes a potentiometer reading and returns a normalized value [0, 1]
    '''
    POT_MAX = 512.0
    POT_MIN = 0.0
    norm_val = (val - POT_MIN) / (POT_MAX - POT_MIN)
    return norm_val


### Setup for main program ###
keys_list = [
    261.63,
    293.66,
    329.63,
    349.23,
    392,
    440,
    493.88,
    523.25
]

# GPIO setup
GPIO.setmode(GPIO.BCM)

# ADC setup
ADC2 = 4
GPIO.setup(ADC2, GPIO.OUT)

ADC3 = 5
GPIO.setup(ADC3, GPIO.OUT)

# frequency button setup
up_freq_button = 23
GPIO.setup(up_freq_button, GPIO.IN, pull_up_down=GPIO.PUD_UP)

down_freq_button = 25
GPIO.setup(down_freq_button, GPIO.IN, pull_up_down=GPIO.PUD_UP)    


# allocate a thread for listening to up button (increase freq)
GPIO.add_event_detect(up_freq_button,
                      GPIO.RISING,
                      callback=increaseFreq,
                      bouncetime=200)
# allocate a thread for listening to down button (decrease freq)
GPIO.add_event_detect(down_freq_button,
                      GPIO.RISING,
                      callback=decreaseFreq,
                      bouncetime=200)

### Main program ###
try:
    
    # set the starting frequency
    freq = keys_list[0]

    # numpy array for storing piezo readings
    nDATA  = 128 # num of datapoints in a packet
    dPIEZO = np.zeros(nDATA,dtype='float')
    
    DIG_THRESH = 615 # signature for digaridoo playing

    # initialize serial peripheral interfaces for potentiometers
    spi1, spi2 = initSpi()
    NUM_POTS = 5 # number of potentiometer values
    pot_vals = [0]*NUM_POTS # for storing potentiometer readings
    
    # instantiate pyAudio output
    out_stream = Output()
   
    while True:
        # get a packet of data from the piezo
        for i in range(nDATA):
            dPIEZO[i] = readPiezo(spi1) #PIEZO element
        # if the data has a signature of the resonating digeridoo
        if np.amax(dPIEZO) > DIG_THRESH:
            # collect all of the potentiometer values
            for i in range(NUM_POTS):
                # get a potentiometer reading
                pot_val = readPot(i, spi1, spi2)
                # normalize it and add it to the list
                pot_vals[i] = normalizePotVal(pot_val)
            #print(pot_vals)
            # generate an output signal
            note = out_stream.note(freq, pot_vals)
            # play the signal
            out_stream.tone(note)
            
except(KeyboardInterrupt, SystemExit):
    print("Interrupt!")

finally:
    print("Done!")
    out_stream.close_audio();          

import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
import ES2EEPROMUtils
from adafruit_mcp3xxx.analog_in import AnalogIn
import threading
from threading import Thread
import datetime
import time
import RPi.GPIO as GPIO
import random

#Declare constants
#=======================================================================
#Pins for ADC and Temperature sensor
nbits = 16
Vref = 3.3 #Volts
Tc = 10 #mV/C
T0 = 500 #mV
button_sample_rate = 6 #Button GPIO port (BCM)
button_stop_start = 26 #GPIO 26
timestep = [10,5,1] #Array of timestep options (static)
#Pins for EEPROM
buzzer = 13
#=======================================================================
#Static global variables to be used by thread and other functions for temperature sensor and ADC
chan = None
runtime = 0
option = 0
start = 0
thread = None
#=======================================================================
#Some global variables that need to change as we run the program
k=None              #instance of pwm object for buzzer 
eeprom = ES2EEPROMUtils.ES2EEPROM()
#=======================================================================
#Main method run on startup.
def main():
    global chan #global variables used in the function
    chan = setup()
    get_time_thread() #Start timer and reading function

#Function to setup GPIO, SPI connection and ADC.
def setup():
    #ADC and Temp sensor setup
    global start
    GPIO.setmode(GPIO.BCM)
    # create the spi bus
    spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)

    # create the cs (chip select)
    cs = digitalio.DigitalInOut(board.D5)

    # create the mcp object
    mcp = MCP.MCP3008(spi, cs)
    # create an analog input channel on pin 0
    chan = AnalogIn(mcp, MCP.P0)
    # Setup Buttons
    GPIO.setup(btn_sample_rate,GPIO.IN,pull_up_down=GPIO.PUD_UP)
    GPIO.setup(button_stop_start,GPIO.IN,pull_up_down=GPIO.PUD_UP)
    # Setup debouncing and callbacks
    GPIO.add_event_detect(button1,GPIO.FALLING,callback=btn_sample,bouncetime=300)
    print("Runtime\t\tTemp Reading\tTemp")
    start=time.time()

    #EEPROM setup
    # Setup PWM channels
    global k
    GPIO.setup(buzzer,GPIO.OUT)
    k=GPIO.PWM(buzzer,1000)
   # k.start(0)
    # Setup debouncing and callbacks
    GPIO.add_event_detect(button_stop_start,GPIO.FALLING,callback=btn_startstop,bouncetime=300)

    return chan #object of analog input channel for ADC returned.

#function to start the thread timer, enable thread daemon, start the thread and then call the function to read the ADC value. This function will iterate every second and check the time elapsed with the chosen timestep. Inititally 10 seconds.
def get_time_thread():
    global chan, option, timestep, start, runtime, thread; #global variables used in function
    runtime = time.time() - start;
    runtime=round(runtime) #Calculate runtime
    thread = threading.Timer(timestep[option], get_time_thread)
    thread.daemon = True #Clean up and close threads on program exit.
    thread.start() #start thread
    read(chan, runtime)

#Function to convert an ADC digital output code to its celcius value.
def ADCToCelcius(ADCcode):
    temp = (((ADCcode * Vref * 1000) / 2**nbits) - T0)/Tc
    return temp

#Function to read channel value from ADC and print to screen.
def read(chan, runtime):
    val = chan.value
    print(runtime, "\t\t", chan.value, '\t\t', str(round(ADCToCelcius(val),3)) + "\tC", sep = '')

#Function to handle GPIO button1 press events
def btn_sample(channel):
    global option, thread
    thread.cancel()
    option += 1
    if option > 2: #timestep has 3 options from 0 to 2.
        option = 0
    get_time_thread()

def btn_startstop(channel):
    print("Liam is a boitjie")

def welcome():
    os.system('clear')
    print("  ______            _                                      _     _                                ")
    print(" |  ____|          (_)                                    | |   | |                               ")
    print(" | |__   _ ____   ___ _ __ ___  _ __  _ __ ___   ___ _ __ | |_  | |     ___   __ _  __ _  ___ _ __ ")
    print(" |  __| | '_ \ \ / / | '__/ _ \| '_ \| '_ ` _ \ / _ \ '_ \| __| | |    / _ \ / _` |/ _` |/ _ \ '__|")
    print(" | |____| | | \ V /| | | | (_) | | | | | | | | |  __/ | | | |_  | |___| (_) | (_| | (_| |  __/ |   ")
    print(" |______|_| |_|\_/ |_|_|  \___/|_| |_|_| |_| |_|\___|_| |_|\__| |______\___/ \__, |\__, |\___|_|   ")
    print("                                                                              __/ | __/ |          ")
    print("                                                                             |___/ |___/           ")

if __name__ == "__main__": #If run as the main script, run main()
    try:
        main()
    except KeyboardInterrupt:
        GPIO.cleanup() #Cleanup GPIO initializations on exit.
    except Exception as e:
        print(e)
        GPIO.cleanup()
    finally:
        GPIO.cleanup()
#=======================================================================
#...............EEE3096S Environment Logger Project....................=
#....................Ian Edwards and Liam McEvoy.......................=
#.....................EDWIAN004 and MCVLIA001..........................=
#=======================================================================
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
import os

#Declare constants
#=======================================================================
#Pins for ADC and Temperature sensor
nbits = 16
Vref = 3.3 #Volts
Tc = 10 #mV/C
T0 = 500 #mV
#button_sample_rate = 6 #Button GPIO port (BCM)
button_stop_start = 6 #GPIO 26
timestep = [10,5,1] #Array of timestep options (static)
#Pins for EEPROM
buzzer = 13
#=======================================================================
#Static global variables to be used by thread and other functions for temperature sensor and ADC
chan = None
runtime = 0
option = 1
start = 0
thread = None
sampleNr = 0
start_stop=None
#=======================================================================
#Some global variables that need to change as we run the program
k=None              #instance of pwm object for buzzer 
eeprom = ES2EEPROMUtils.ES2EEPROM()
#=======================================================================
#Main method run on startup.
def main():
    global chan, thread; #global variables used in the function
    chan = setup(); # Call setup function to setup ADC, and constants.
    log_temps() #Start timer and reading function
    while True: # Loop infinitely allowing for continuous logging.
        pass

#Function to setup GPIO, SPI connection and ADC.
def setup():
    #ADC and Temp sensor setup
    global chan, start, start_stop, k
    GPIO.setmode(GPIO.BCM)
    # create the spi bus
    spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)
    # create the cs (chip select)
    cs = digitalio.DigitalInOut(board.D5)
    # create the mcp object
    mcp = MCP.MCP3008(spi, cs);
    # create an analog input channel on pin 0
    chan = AnalogIn(mcp, MCP.P0);
    # Setup Button
    GPIO.setup(button_stop_start,GPIO.IN,pull_up_down=GPIO.PUD_UP);
    # Setup debouncing and callbacks
   # GPIO.add_event_detect(button_stop_start,GPIO.FALLING,callback=btn_startstop,bouncetime=300);
    start=time.time();
    start_stop=1;
    print("Time:\t\t\tRuntime\t\tTemperature:")
    #EEPROM setup
    # Setup PWM channels
    GPIO.setup(buzzer, GPIO.OUT)
    k=GPIO.PWM(buzzer,1000)
    k.start(0)
    # Setup debouncing and callbacks
    GPIO.add_event_detect(button_stop_start,GPIO.FALLING,callback=btn_startstop,bouncetime=300)
    return chan #object of analog input channel for ADC returned.

#Function to convert an ADC digital output code to its celcius value.
def ADCToCelcius(ADCcode):
    temp = (((ADCcode * Vref * 1000) / 2**nbits) - T0)/Tc
    return temp

#Function to read channel value from ADC and print to screen.
def read(chan):
    global sampleNr, start_stop, start; #global variables used in function
    if(start_stop==1):
        #Get ADC value for temperature
        val = chan.value 
        temp = int(ADCToCelcius(val))
        currTime = datetime.datetime.now().time()
        pTime = format_time(currTime) #Format time for printing.
        store_temps(temp, currTime) #Call method to store temperature and time value in EEPROM.
        runtime = int(time.time() - start)
        print(pTime, "\t\t", runtime, '\t\t', str(temp) + " C", sep = '', end = '')
        
        sampleNr += 1
        if (sampleNr % 5 == 0 and start_stop ==1): # If it has been 5 samples, trigger the buzzer only if the button has not been toggled.
            trigger_buzzer(1)
            print('*')
        else:
            trigger_buzzer(0)
            print()
        
    else:
        sampleNr=0;

def retrieve_temps(): # Function reads values from EEPROM.
    readings_count = eeprom.read_byte(0) # Number of samples stored is stored in first byte.
    temp_readings = []
    for i in range(1, readings_count+1):
        temp_readings.append(eeprom.read_block(i,4)) # Read 4 bytes from EEPROM and append as a list to a list.
    return readings_count, temp_readings # Return a list

def store_temps(temp, currTime):
    #Read EEPROM
    t_count, t_readings = retrieve_temps()
    # If 20 samples are stored, delete oldest sample (first stored), else increase number of values stored.
    if t_count < 20:
        t_count += 1
    elif t_count >=20:
        del t_readings[0]
    t_readings.append([currTime.hour, currTime.minute, currTime.second, temp]) #Array of format [[hour,min,sec,temp],[etc],[],[]]
    data_to_write = [t_count,0,0,0] #Add initial block for storing number of values in EEPROM.
    for reading in t_readings:
        for k in range(0,4):
            data_to_write.append(reading[k]) #Add hour,min,second and temp to array to be written, per entry in t_readings: [NrEntries,0,0,0,hour,min,second,temp,hour,min,second,temp, etc]
    eeprom.write_block(0, data_to_write) #Write array to EEPROM

def print_temps(): #Function to retrieve and print first 20 contents of EEPROM.
    samples = retrieve_temps()
    print("Number of samples stored: ", samples[0])
    print("Samples:\n", samples[1])
    
#Function to start the thread timer, enable thread daemon, start the thread and then call the function to read the ADC value. 
#This function will iterate every 5 seconds and call read all as a separate thread from the main thread that loops infinitely.
def log_temps():
    global chan #global variables used in function
    thread_data = threading.Timer(timestep[1], log_temps) # Length of timer can be changed.
    thread_data.daemon = True #Clean up and close threads on program exit.
    thread_data.start() #start thread
    read(chan) #read ADC and store to EEPROM.

def format_time(currTime): #Format time for printing.
    if currTime.second < 10:
        seconds = "0" + str(currTime.second) #Formatting to align in columns
    else: seconds = str(currTime.second)

    if currTime.hour < 10:
        hours = "0" + str(currTime.hour) #Formatting to align in columns
    else: hours = str(currTime.hour)

    if currTime.minute < 10:
        minutes = "0" + str(currTime.minute) #Formatting to align in columns
    else: minutes = str(currTime.minute)

    return hours + ":" + minutes + ":" + seconds

def btn_startstop(channel):#Stops/Starts sensor monitoring, but thread is unaffected.
    global start_stop, thread, sampleNr, runtime, start;
    if (start_stop==1):
        start_stop=0; #Toggle start_stop condition. (Stop)
        sampleNr = 0; #Reset sampleNr so buzzer will buzz in next 5 samples time.
        trigger_buzzer(0); #Turn off buzzer.
        welcome() #Redisplay (clear screen)
        print("Logging has stopped. Press button to start logging again.");
    else: #Press again to start again.
        sampleNr = 0;
        trigger_buzzer(0);
        welcome()
        print("Logging has started.");
        print("Time:\t\t\tRuntime\t\tTemperature:")
        start_stop=1;
        start = time.time(); #Read() has runtime = time.time() - start

def welcome(): # Extra fun ASICC graphic welcome message.
    os.system('clear')
    print("  ______            _                                      _     _                                ")
    print(" |  ____|          (_)                                    | |   | |                               ")
    print(" | |__   _ ____   ___ _ __ ___  _ __  _ __ ___   ___ _ __ | |_  | |     ___   __ _  __ _  ___ _ __ ")
    print(" |  __| | '_ \ \ / / | '__/ _ \| '_ \| '_ ` _ \ / _ \ '_ \| __| | |    / _ \ / _` |/ _` |/ _ \ '__|")
    print(" | |____| | | \ V /| | | | (_) | | | | | | | | |  __/ | | | |_  | |___| (_) | (_| | (_| |  __/ |   ")
    print(" |______|_| |_|\_/ |_|_|  \___/|_| |_|_| |_| |_|\___|_| |_|\__| |______\___/ \__, |\__, |\___|_|   ")
    print("                                                                              __/ | __/ |          ")
    print("                                                                             |___/ |___/           ")

#turn buzzer on
def trigger_buzzer(boolean):
    if(boolean==1):
        k.ChangeDutyCycle(0.5)
        k.ChangeFrequency(1)
    else:
        k.ChangeDutyCycle(0) #turns buzzer off
    
if __name__ == "__main__": #If run as the main script, run main()
    try:
        welcome()
        main()
    except KeyboardInterrupt as e:
        print_temps()
        GPIO.cleanup() #Cleanup GPIO initializations on exit.
        print(e)
    except Exception as e:
        print(e)
        GPIO.cleanup()
    finally:
        GPIO.cleanup()
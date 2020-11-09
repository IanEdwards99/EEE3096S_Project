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
    global chan,start_stop, thread; #global variables used in the function
    chan = setup();
    while True:
        print("1) Start environment monitoring.\n2) View EEPROM.")
        option = input("Choose an option from above:\n")
        if option == "1":
            print("Runtime\t\t\tTime\t\tTemp Reading");
            start_stop=1;
            get_time_thread(); #Start timer and reading function
            try:
                while True: #loop infinitely so thread can run and print ADC values to display. Note: timer does printing so ADC is only read and printed to display each timestep.
                    pass
            except KeyboardInterrupt:
                print("Monitoring stopped.")
                thread.cancel()
                start_stop = 0;
                trigger_buzzer(0)

        elif option == "2":
            arrResult = fetch_temp()
            print(arrResult[1])

#Function to setup GPIO, SPI connection and ADC.
def setup():
    #ADC and Temp sensor setup
    global start, chan
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

    #EEPROM setup
    # Setup PWM channels
    global k
    GPIO.setup(buzzer, GPIO.OUT)
    k=GPIO.PWM(buzzer,1000)
    k.start(0)
    # Setup debouncing and callbacks
    GPIO.add_event_detect(button_stop_start,GPIO.FALLING,callback=btn_startstop,bouncetime=300)
    return chan #object of analog input channel for ADC returned.

#function to start the thread timer, enable thread daemon, start the thread and then call the function to read the ADC value. This function will iterate every second and check the time elapsed with the chosen timestep. Inititally 10 seconds.
def get_time_thread():
    global chan, option, timestep, start, runtime, thread; #global variables used in function
    runtime = time.time() - start;
    runtime=round(runtime) #Calculate runtime
    thread = threading.Timer(timestep[option], get_time_thread)
    thread.daemon = True; #Clean up and close threads on program exit.
    thread.start() #start thread
    if (start_stop == 1):
        read(chan, runtime)
    else:
        sampleNr = 0;
        trigger_buzzer(0)

#Function to convert an ADC digital output code to its celcius value.
def ADCToCelcius(ADCcode):
    temp = (((ADCcode * Vref * 1000) / 2**nbits) - T0)/Tc
    return temp

#Function to read channel value from ADC and print to screen.
def read(chan, runtime):
    global sampleNr, start_stop;
    val = chan.value 
    temp = round(ADCToCelcius(val),3)
    if(start_stop==1):
        currentTime = datetime.datetime.now().time()
        timeArr = [currentTime.hour, currentTime.minute, currentTime.second]
        save_temp(timeArr, temp)
        if timeArr[2] < 10: #Format 1 to 01.
            timeArr[2] = "0"+str(timeArr[2])
        
        if (start_stop==1):
            print(timeArr[0],":", timeArr[1],":", timeArr[2], "\t\t", runtime, '\t\t', str(temp) + " C", sep = '')
            sampleNr += 1
        if (sampleNr % 5 == 0):
            trigger_buzzer(1)
        else:
            trigger_buzzer(0)
    else:
        sampleNr=0;


def btn_startstop(channel):#Stops/Starts sensor monitoring, but thread is unaffected.
    global start_stop, thread, sampleNr, runtime;
    if (start_stop==1):
        start_stop=0;
        runtime = 0;
        sampleNr = 0;
        trigger_buzzer(0);
        welcome()
        print("Logging has stopped. Press Buzzer to start logging again.");
        print();
        trigger_buzzer(0);
    else:
        sampleNr = 0;
        runtime = 0;
        trigger_buzzer(0);
        welcome()
        print("Logging has started.");
        print("Runtime\t\t\tTime\t\tTemp");
        start_stop=1;
        trigger_buzzer(0);

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

# Load temp
def fetch_temp():
    # get however many temp there are
    temp_count = eeprom.read_byte(0)
    print(temp_count)
    tempscores=eeprom.read_block(1,temp_count*4)
    # Get the temperatures and time
    temperature=[]
    for number in range(temp_count):
        temp=[]
        temptime=[]
        temptime.append(tempscores.pop(0)) #hour
        temptime.append(tempscores.pop(0)) #minute
        temptime.append(tempscores.pop(0)) #second
        temp.append(temptime)
        temp.append(tempscores.pop(0)) # temperature
        temperature.append(temp)
    
    # return back the results
    return temp_count, temperature #temperature=[  [hour,minute,second],temp]  ,   [hour,minute,second],temp]   ,   [hour,minute,second],temp] ]

def write_temp(t_count,temp_readings):
    eeprom.clear(4096)
    eeprom.write_block(0, [t_count])
    data_to_write = []
    for reading in temp_readings:  #temp_readings in format[ [ [hour,minute,second] , [temp] ]   ,   [ [hour,minute,second] , [temp] ]   ,   [ [hour,minute,second] , [temp] ] ]
        data_to_write.append(reading[0][0]) #hour
        data_to_write.append(reading[0][1]) #minute
        data_to_write.append(reading[0][2]) #second 
        data_to_write.append(int(reading[1])) #temperature
    eeprom.write_block(1, data_to_write)

# Save temperature
def save_temp(time,temperature):
    # fetch temp            time is an array [hour,minute,second]
    t_count, temp_time=fetch_temp()
    if (t_count==20):
        temp_time.pop(0)
        temp_time.append([time,temperature])
    else:
        t_count=t_count+1
        temp_time.append([time,temperature])
        
    write_temp(t_count,temp_time)

#turn buzzer on
def trigger_buzzer(boolean):
    if(boolean==1):
        k.ChangeDutyCycle(0.5)
        k.ChangeFrequency(1)
    else:
        k.ChangeDutyCycle(0)
    


if __name__ == "__main__": #If run as the main script, run main()
    try:
        welcome()
        main()
    except KeyboardInterrupt:
        GPIO.cleanup() #Cleanup GPIO initializations on exit.
    except Exception as e:
        print(e)
        GPIO.cleanup()
    finally:
        GPIO.cleanup()
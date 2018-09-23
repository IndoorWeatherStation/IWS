import time,pytz,Adafruit_MCP3008,VL53L0X,urllib2,serial
import datetime as dt
from Adafruit_BME280 import *
from sgp30 import SGP30
from smbus import SMBus
##import Adafruit_IO

############################## SETUP ##############################
###General setup
smbus = SMBus(1)
lastmin = dt.datetime.now().minute
mcp = Adafruit_MCP3008.MCP3008(clk=18,cs=25,miso=23,mosi=24)
res = 60.0 #How often to upoad data, in this case every minute
sums = [0,0,0,0,0,0,0,0] #Here is where we will add up all measurements per minute for each sensor
num = 0 #Count the number of times we go through the loop each minute so we can calculate average values
timeZone = "Europe/Berlin" #Choose from here https://gist.github.com/heyalexej/8bf688fd67d7199be4a1682b3eec7568

#####Adafruit IO setup
##io = Adafruit_IO.Client('YOUR_USERNAME','YOUR_KEY')
##try: #Check if we have internet connectivity
##    urllib2.urlopen('http://www.google.com').close()
##    internet = True
##except: internet = False
##if internet:
##    f0 = io.feeds('all')
##    f1 = io.feeds('temperature')
##    f2 = io.feeds('humidity')
##    f3 = io.feeds('wind')
##    f4 = io.feeds('luminosity')
##    f5 = io.feeds('noise')
##    f6 = io.feeds('co2')
##    f7 = io.feeds('voc')
##    f8 = io.feeds('distance')
##else: print("No internet")

###BME280
try: bme = BME280(t_mode=BME280_OSAMPLE_8, p_mode=BME280_OSAMPLE_8, h_mode=BME280_OSAMPLE_8)
except: print('Something wrong with BME280')

###TSL2561
try:
    smbus.write_byte_data(0x39, 0x00 | 0x80, 0x03)
    smbus.write_byte_data(0x39, 0x01 | 0x80, 0x02)
    time.sleep(0.5)
except: print('Something wrong with TSL2561')

###VL53L0X
try:
    tof = VL53L0X.VL53L0X()
    tof.start_ranging(VL53L0X.VL53L0X_BETTER_ACCURACY_MODE)
except: print('Something wrong with VL53L0X')

###S8
ser = serial.Serial("/dev/ttyS0",baudrate =9600,timeout = .5)
ser.flushInput()
time.sleep(1)

###MAX4466
sampleWindow = 1
    
###Rev C
analogPinForTMP = 1
analogPinForRV = 2
zeroWindAdjustment =  0 #####Calibrate the Rev C to find out this value#####

############################# LOOP #############################
with SGP30(smbus) as chip:
    while True:
        ###Set initial values
        tim,T,H,W,L,N,C,V,D = (dt.datetime.now(tz=pytz.timezone(timeZone)),0,0,0,0,0,0,0,0)
        
        ###BME280
        try:
            if not bme:
                try: bme = BME280(t_mode=BME280_OSAMPLE_8, p_mode=BME280_OSAMPLE_8, h_mode=BME280_OSAMPLE_8)
                except: pass
            T = bme.read_temperature()
            H = bme.read_humidity()
        except: print('Problem with BME280')

        ###TSL2561
        try:
            data = smbus.read_i2c_block_data(0x39, 0x0C | 0x80, 2)
            data1 = smbus.read_i2c_block_data(0x39, 0x0E | 0x80, 2)
            ch0 = data[1] * 256 + data[0]
            ch1 = data1[1] * 256 + data1[0]
            L = ch0-ch1
        except: print('Problem with TSL2561')
        
        ###VL53L0X
        start = time.time()
        try: D = tof.get_distance()
        except: print('Problem with VL53L0X')
        if (time.time()-start)>2:
            try:
                tof.stop_ranging()
                tof = VL53L0X.VL53L0X()
                tof.start_ranging(VL53L0X.VL53L0X_BETTER_ACCURACY_MODE)
            except: print('Problem with VL53L0X')
        
        ###SGP30
        try:
            aq = chip.measure_air_quality()
            V = aq.voc_ppb
        except: print('Problem with SGP30')
        
        ###MAX4466
        try:
            signalmax = 0
            signalmin = 1024
            startloop = time.time()
            ncount = 0
            while (time.time()-startloop)<sampleWindow:
                sample = mcp.read_adc(0)
                if sample<1024:
                    if sample>signalmax: signalmax = sample
                    if sample<signalmin: signalmin = sample
            peaktopeak = signalmax-signalmin
            N = (peaktopeak*3.3)/1024
        except: print('Problem with MAX4466')
        
        ###S8
        try:
            ser.flushInput()
            ser.write('\xFE\x44\x00\x08\x02\x9F\x25')
            time.sleep(1)
            resp = ser.read(7)
            C = (ord(resp[3])*256) + ord(resp[4])
        except: print('Problem with S8')
        
        ###Rev C
        try:
            TMP_Therm_ADunits = mcp.read_adc(analogPinForTMP)
            RV_Wind_ADunits = mcp.read_adc(analogPinForRV)
            RV_Wind_Volts = (RV_Wind_ADunits * 0.0048828125)
            TempCtimes100 = (0.005 * (float(TMP_Therm_ADunits) * float(TMP_Therm_ADunits))) - (16.862 * float(TMP_Therm_ADunits)) + 9075.4
            zeroWind_ADunits = -0.0006 * (float(TMP_Therm_ADunits) * float(TMP_Therm_ADunits)) + 1.0727 * float(TMP_Therm_ADunits) + 47.172
            zeroWind_volts = (zeroWind_ADunits * 0.0048828125) - zeroWindAdjustment 
            try: WindSpeed_MPH =  pow(((RV_Wind_Volts - zeroWind_volts) /.2300) , 2.7265)
            except:
                WindSpeed_MPH = 0.0
                print('Rev C needs calibration')
            W = float(WindSpeed_MPH*0.44704)
        except: print('Problem with Rev C')
        
        ###Add each value to sums to we can later take the average by minute
        sums[0]+=float(T)
        sums[1]+=float(H)
        sums[2]+=float(W)
        sums[3]+=float(L)
        sums[4]+=float(N)
        sums[5]+=float(C)
        sums[6]+=float(V)
        sums[7]+=float(D)
        num+=1
        
        ###Print to terminal
        if tim!='None': temptim = tim.strftime('%y.%m.%d %H:%M:%S')
        else: temptim = tim
        if T!=0: T = round(T,2)
        if H!=0: H = round(H,2)
        if W!=0: W = round(W,2)
        if L!=0: L = round(L)
        if N!=0: N = round(N,2)
        if C!=0: C = round(C)
        if V!=0: V = round(V)
        if D!=0: D = round(D)
        print('Time: %s  |  T: %.2f  |  H: %.2f  |  W: %.2f  |  L: %d  |  N: %.2f  |  C: %d  |  V: %d  |  D: %d'%(temptim,T,H,W,L,N,C,V,D))
        
        ###Write line every minute with the average values
        if dt.datetime.now().minute!=lastmin: #Check whether a line has already been written this minute
            T = round(sums[0]/num,2)
            H = round(sums[1]/num,2)
            W = round(sums[2]/num,2)
            L = int(round(sums[3]/num))
            N = round(sums[4]/num,2)
            C = int(round(sums[5]/num))
            V = int(round(sums[6]/num))
            D = int(round(sums[7]/num))
            line = '%s,%.2f,%.2f,%.2f,%.d,%.2f,%.d,%.d,%.d\n'%(tim,T,H,W,L,N,C,V,D)
            print('%s'%line[:-1])
            with open('/home/pi/DataLogging/DATA/'+str(str(dt.date.today()))+'.csv','a') as log: log.write(line)
            lastmin = dt.datetime.now().minute
            sums = [0,0,0,0,0,0,0,0]
            num = 0
            
##            ###Write to Adafruit IO
##            try:
##                urllib2.urlopen('http://www.google.com').close()
##                internet = True
##            except urllib2.URLError:
##                internet = False
##                print('\nNo internet connection')
##            if internet:
##                try: a = f0.key
##                except:
##                    f0 = io.feeds('all')
##                    f1 = io.feeds('temperature')
##                    f2 = io.feeds('humidity')
##                    f3 = io.feeds('wind')
##                    f4 = io.feeds('luminosity')
##                    f5 = io.feeds('noise')
##                    f6 = io.feeds('co2')
##                    f7 = io.feeds('voc')
##                    f8 = io.feeds('distance')
##                try:
##                    io.send(f0.key,line[:-1])
##                    io.send(f1.key,T)
##                    io.send(f2.key,H)
##                    io.send(f3.key,W)
##                    io.send(f4.key,L)
##                    io.send(f5.key,N)
##                    io.send(f6.key,C)
##                    io.send(f7.key,V)
##                    io.send(f8.key,D)
##                except Adafruit_IO.errors.ThrottlingError:
##                    #Just in case for some reason we exceed Adafruit's maximum rate, wait a bit
##                    print('ThrottlingError')
##                    time.sleep(30)
tof.stop_ranging()

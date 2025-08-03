For an aggracultural application of LoraWAN, I found that multiple sensor types are required, and rather than building PCBs to handle each case, I wanted to have a sensor board that could manage all the uses that I needed, understanding that no single node would use all resources.  

re-485 : isolated with an isolated 5 v power source
I2C : for temp and  humidity sensors that favor this.
current loop : for lake depth sensors that use 4-20 ma 12 or 24V current loops.
reed counter : water flow sensors to monitor flow volume and rate, using reed switch output, this is monitored in the Ultra Low Power (ULP) processor and continues in deep sleep of the MCU.
DC latching solenoids : motor drivers are used to actuate 9v or 12v DC latching solenoids to manage valves.  Two valves per sensor board.  
ADC : used to monitor water line pressure or for the current loop.  18 bit precision on a 2 v range with i2c data.  
solar charge controller for 6v (small) solar panel to maintain the battery.

I am using the Heltec e290 board, as the eink screen is very low power and in deep sleep with the ULP running uses about 150 uAmp, reduced to 25 uAmp if the ULP is not  started.

I used JLCBCB.  The EasyEdaPro files for this board are included.  The board is connected to the Helted e290 with a 40 pin male/female 20 cm cable.  

Board configuration:
The RS-485 including the 5v isolated supply is engagued by shorting its pins.
The voltage of the boost regulator is set with shorts to pins, 24v, 12v, or both 9v.
Non populated resistor pads can be shorted to change the time constant of the reed debouncer from 10 ms as built to 0.9 ms by shorting that pad.  
The RS-485 termination resistor can be bypassed by shorting the center pad of the RS-485 region.  
RS-485 bias resistors (1kOhm) can be included by shorting the lateral pads of the RS-485 region.  

To deploy:
1100 mAmp battery
6v 120 mAmp solar panel.
Heltec-e290 with antenna (I used the standard SMC cable to the micro adapter to penetrate the project box).
project box with see though lid to see the eink display.
grommet to penetrate the box with the sensor cable.
web serever running chirpstack 4 in docker as well as flask, mySQL.

I have used the v4 of chirpstack as the docker image on EC2, and added mySQL to manage the data.  The data is then served using python.  I will upload that once modifications are complete.  

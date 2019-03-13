# This program will let you test your ESC and brushless motor.
# Make sure your battery is not connected if you are going to calibrate it at first.

from time import sleep, time
import os
os.system ("sudo pigpiod")
sleep(1) #Wait for pigpiod to initialise
import pigpio


class PairESCController:
    def __init__(self, pins=(23,21)):
        self.ESC, self.ESC2 = pins

        pi = pigpio.pi()
        self.stop()
        self.maxValue = 2000
        self.minValue = 1000

        self.calibrated = False


    def manual_drive(self, duty, debug=True, doNotCalibrate=False):
        #Set the ESC PWM duty
        if debug:
            print("Setting the motors to duty: {} (bigger is faster, {}<duty<{})".format(duty, self.minValue, self.maxValue))

        if self.calibrated == False and doNotCalibrate == False:
            self.calibrate(test=False)

        pi.set_servo_pulsewidth(self.ESC,duty)
        pi.set_servo_pulsewidth(self.ESC2,duty)

    def calibrate(self, test=True):
        #Calibrate the ESC to allow it to drive
        #Note that this almost certainly has uncessary sleeps in

        self.manual_drive(0, debug=False) #self.stop()
        print("Disconnect the battery and press Enter")
        inp = raw_input() #Add code to do relay connect/disconnect instead

        self.manual_drive(self.maxValue, debug=False)
        print("Connect the battery now, you will here two beeps, then wait for a gradual falling tone then press Enter")
        inp = raw_input() #Add sleep as necessary instead

        self.manual_drive(self.minValue, debug=False)

        #Do we even need this step? We've already done it before
        if 1:
            print("You should another tone from every motor")
            for i in range(13):
                print("{} seconds till next process (note, we can probably reduce this)".format(13-i))
            self.manual_drive(0, debug=False) #self.stop()

        self.calibrated = True
        sleep(2)

        if test:
            #print("Arming ESC now")
            self.manual_drive(self.minValue, debug=False)
            print("Motors spinning up for 10 seconds at the lowest speed")
            sleep(10) # You can change this to any other function you want
            print("Motors spinning down, and stopping")
            self.stop()

    def stop(self):
        #Stop the ESCs
        self.manual_drive(0, debug=False, doNotCalibrate=False)

    def stopstop(self):
        #Stop the ESCs and kill the pigpiod daemons
        self.stop()
        pi.stop()

if __name__ == "__main__":
    c = PairESCController()
    c.manual_drive(1500)

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

        self.pi = pigpio.pi()
        #self.stop()
        self.maxValue = 2000
        self.minValue = 1000

        self.calibrated = False


    def manual_drive(self, duty, debug=False, doNotCalibrate=True):
        """Set drive both motors, by setting their PWM duties
        Parameter 1: duty [int]; the PWM duty of both flywheels
        Optional parameter 2: debug [boolean]; whether a debug message should be printed
        Optional parameter 3: doNotCalibrate [boolean]; whether the motors should automatically calibrate themselves
        """
        if debug:
            print("Setting the motors to duty: {}".format(duty))
            #print("Setting the motors to duty: {} (bigger is faster, {}<duty<{})".format(duty, self.minValue, self.maxValue))

        if doNotCalibrate == False and self.calibrated == False:
            self.calibrate()

        self.pi.set_servo_pulsewidth(self.ESC,duty)
        self.pi.set_servo_pulsewidth(self.ESC2,duty)

    def calibrate(self):
        """Calibrate the ESCs to allow them to drive
        """

        self.stop()
        print("Disconnect the battery and press Enter")
        inp = input() #Add code to do relay connect/disconnect instead

        self.manual_drive(self.maxValue, debug=False)
        print("Connect the battery now, you will here two beeps, then wait for a gradual falling tone then press Enter")
        inp = input() #Add sleep as necessary instead

        self.manual_drive(self.minValue, debug=False)

        print("You should another tone from every motor")
        for i in range(13):
            if i%5==0:
                print("{} seconds till next process".format(13-i))
            sleep(1)
        self.stop()
        self.calibrated = True
        sleep(2)

        #print("Arming ESC now")
        self.manual_drive(self.minValue, debug=False)
        print("Motors spinning up for 10 seconds at the lowest speed")
        sleep(10) # You can change this to any other function you want
        print("Motors spinning down, and stopping")
        self.stop()

    def stop(self):
        """Stop the ESCs"""
        self.manual_drive(0, debug=False)

    def stopstop(self):
        """Stop the ESCs and kill the pigpiod daemons"""
        self.stop()
        self.pi.stop()

if __name__ == "__main__":
    c = PairESCController()
    c.manual_drive(1130)

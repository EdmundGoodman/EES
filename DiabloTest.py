import Diablo#3 as Diablo
import time

print(Diablo.ScanForDiablo)

m = Diablo.Diablo()
m.i2cAddress = 0x25
m.Init()
m.SetMotors(-1)
time.sleep(.5)
m.SetMotors(0.3)
time.sleep(3)
m.SetMotors(0)

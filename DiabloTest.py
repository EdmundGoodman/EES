if 1:
    import Diablo3 as Diablo
else:
    import Diablo
import time

print(Diablo.ScanForDiablo())

m = Diablo.Diablo()
m.i2cAddress = 0x25
m.Init()
m.SetMotors(-1)
time.sleep(.75)
m.SetMotors(0.3)
time.sleep(1)
m.SetMotors(0)

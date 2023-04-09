import u12
import time

# 작성자 : KETI 신장범 / 문의 : roy_shin@keti.re.kr
# 목적 : labjack 을 이용해서 하이브리드 그리퍼의 공압 환경을 컨트럴

class KetiSuction():
    def __init__(self):
        self.d = u12.U12()

    def suction_up(self):
        self.d.eAnalogOut(analogOut0=2.0, analogOut1=2.0)
        #self.d.eAnalogOut(analogOut0=0.0, analogOut1=0.0)
        time.sleep(1)
        print('suction up')

    def suction_down(self):
        self.d.eAnalogOut(analogOut0=0.0, analogOut1=0.0)
        #self.d.eAnalogOut(analogOut0=2.0, analogOut1=2.0)
        time.sleep(1)
        print('suction down')

    def suction_on(self):
        #self.d.eAnalogOut(analogOut0=0.0, analogOut1=2.0)
        self.d.eAnalogOut(analogOut0=2.0, analogOut1=2.0)
        time.sleep(1)
        #print('suction on')

    def suction_off(self):
        #self.d.eAnalogOut(analogOut0=2.0, analogOut1=0.0)
        self.d.eAnalogOut(analogOut0=0.0, analogOut1=2.0)
        time.sleep(1)
        #print('suction off')

    def suction_end(self):
        self.d.close()
        time.sleep(1)

    def getSignal(self,channel=0):
        print(self.d.eAnalogIn(0))
        print(self.d.eAnalogIn(1))
        print(self.d.eAnalogIn(2))
        print(self.d.eAnalogIn(3))
        print(self.d.eAnalogIn(4))
        print(self.d.eAnalogIn(5))
        print(self.d.eAnalogIn(6))
        print(self.d.eAnalogIn(7))
        print(self.d.eAnalogIn(8))
        print(self.d.eAnalogIn(9))




if __name__ == '__main__':
    a = KetiSuction()
    a.suction_on()
    time.sleep(1)
    a.suction_off()
import serial
import time
import serial.tools.list_ports
import sys


print("ПРОГРАММА КАЛИБРОВКИ ДАТЧИКОВ ВЕСА\n")

ports = serial.tools.list_ports.comports() # список устройств (весовых индикаторо), подключенных через COM-порт к компьютеру
arduinoData = serial.Serial() # создаем объект класса Serial

portList = [] # будущий список весовых индикаторов с другим форматом отображения порта (через метод str())

print("Список устройств, которые подключены к данному ПК через COM port")

for onePort in ports:   # Цикл, который берет порты из ports и затем
    portList.append(str(onePort))
    print(str(onePort))

print(" ")

if len(portList) == 0:
    print("Нет устройств, подключенных к данному ПК. Программа выключится через несколько секунд. Подключите индикатор и перезапустите программу калибровки")
    time.sleep(10)
    sys.exit()

while True:
    val = input("Выберите СОМ порт, к которому подключен индикатор GoCart (USB-SERIAL CH340): COM")
    try:
        int(val)
    except ValueError:
        print("Неверный формат! \n")
        continue
    break

for x in range(0, len(portList)):
    if portList[x].startswith("COM" + str(val)):
        portVar = "COM" + str(val)
        print(f"Вы выбрали {portList[x]}. Идет подключение к индикатору, процесс займет несколько секунд \n")

arduinoData.baudrate = 9600


try:
    arduinoData.port = portVar
except NameError:
    print("\nНеверный СОМ порт. Программа выключится через несколько секунд. Перезапустите программу калибровки")
    time.sleep(5)
    sys.exit()


arduinoData.open()
ans = "as"

com = "ghjjhg789hfk!hgfka7JHB45"
com = com + '\r'

while ans != "ACCEPTED!":
    arduinoData.write(com.encode())
    while arduinoData.in_waiting == 0:
        time.sleep(0.1)
        continue

    if arduinoData.in_waiting > 0:
        ans = arduinoData.readline().decode('utf-8').rstrip()

    time.sleep(0.1)

print("ПРОГРАММА КАЛИБРОВКИ ДАТЧИКА ГОТОВА:")

while True:
    makezero = input("ШАГ 1 - ОБНУЛЕНИЕ: Уберите груз с поверхности платформы, после чего введите '1': ")
    if makezero != "1":
        print("Неверная команда!")
    else:
        break
makezero = makezero + '\r'
arduinoData.write(makezero.encode())

while True:
    if arduinoData.in_waiting > 0:
        ans = arduinoData.readline().decode('utf-8').rstrip()
        if ans == "STEP 1 COMPLETED":

            while True:
                etalon_weight = input("ШАГ 2 - ЭТАЛОННЫЙ ВЕС. Введите значение эталонного веса и затем нажмите enter. ")
                try:
                    float(etalon_weight)
                except ValueError:
                    print("Неверный формат!")
                    continue

                break
            etalon_weight = etalon_weight + '\r'
            arduinoData.write(etalon_weight.encode())
            break

while True:
    if arduinoData.in_waiting > 0:
        ans = arduinoData.readline().decode('utf-8').rstrip()
        if ans == "STEP 2 COMPLETED":
            while True:
                discrete = input("ШАГ 3 - ДИСКРЕТА: Введите значение дискреты: ")
                try:
                    int(discrete)
                except ValueError:
                    print("Неверный формат!")
                    continue

                break

            discrete = discrete + '\r'
            arduinoData.write(discrete.encode())
            break

while True:
    if arduinoData.in_waiting > 0:
        ans = arduinoData.readline().decode('utf-8').rstrip()
        if ans == "STEP 3 COMPLETED":
            while True:
                max_weight = input("ШАГ 4 - Максимальная грузоподъемность. Введите значение максимальной грузоподъемности: ")
                try:
                    int(max_weight)
                except ValueError:
                    print("Неверный формат!")
                    continue
                break

            max_weight = max_weight + '\r'
            arduinoData.write(max_weight.encode())
            break

while True:
    if arduinoData.in_waiting > 0:
        ans = arduinoData.readline().decode('utf-8').rstrip()
        if ans == "FINISH":
            print("Калибровка завершена!")
            break

i = 0

while i < 40:
    if arduinoData.in_waiting > 0:
        ans = arduinoData.readline().decode('utf-8').rstrip()
        print(ans)
        i = i + 1

arduinoData.close()

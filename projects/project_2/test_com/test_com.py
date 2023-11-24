import serial.tools.list_ports
import serial


connect = 0
while connect == 0:
    ports = serial.tools.list_ports.comports()
    for port in ports:
        print(port.device)

    COM_PORT = input("Введите номер COM порта из перечисленных выше или неправильное значение для повторного вывода COM портов: ")

    for port in ports:
        if (COM_PORT == port.device) :
            connect = 1
   
print(COM_PORT)
port = COM_PORT
baudrate = 115200  

ser = serial.Serial(port, baudrate)

number_of_print = 0

while(1):
    data = int.from_bytes(ser.read(2), "little")
    if (number_of_print % 100) == 0:
        print('%d : %d\n' % (number_of_print, data))
        number_of_print = 0    
    number_of_print += 1
# Perform operations on the COM port

ser.close()  # Remember to close the connection when done
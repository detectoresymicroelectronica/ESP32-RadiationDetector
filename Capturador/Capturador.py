from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
from datetime import datetime
import numpy as np
import argparse
import _thread
import serial
import time
import os

#In[0]: Parametros
parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)

parser.add_argument('-p', '--path',   type=str, action="store", dest='p', default="Mediciones",   help='Directorio de trabajo')
parser.add_argument('-n', '--name',   type=str, action="store", dest='n', default="Aluminum",     help='Nombre de archivo')
parser.add_argument('-c', '--cant',   type=int, action="store", dest='c', default=1000,           help='Cantidad de frames')
parser.add_argument('-s', '--serial', type=str, action="store", dest='s', default="/dev/ttyUSB0", help='Puerto serie')

args = parser.parse_args()

#In[1]: Variables globales

#Puerto serie
ser = serial.Serial(
	port=args.s,
	baudrate="115200",
	parity=serial.PARITY_NONE,
	stopbits=serial.STOPBITS_ONE,
	bytesize=serial.EIGHTBITS,
	xonxoff=0,
	rtscts=0,
	dsrdtr=0,
	timeout=5.0
)

#Calibracion en energia
canales  = [134, 148]   #Canales de lineas de Cu
energias = [8.05, 8.91] #Energias de lineas de Cu

m = ( energias[1] - energias[0]) /  (canales[1] - canales[0])
b = energias[0] - m*canales[0]

chns = np.arange(256)
engs = chns*m + b

xmin, xmax = -1, 255
emin, emax = xmin*m + b, xmax*m + b

#Histograma
ys = np.zeros(256, dtype=np.uint64)

#Extras
date = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
anim = 0

#Crear directorio para almacenar 
if not os.path.isdir(f'{args.p}'):
    os.mkdir(f'{args.p}')

#In[1]: Funciones utiles
def Leer_Sensor(puerto):
    hist0 = puerto.read_until(b' \r\n')
    hist = str(hist0, 'utf-8')
    hist = hist.split()

    return hist

def Thread_Capturar():
    global ys

    errr_cam = 0
    errr_ser = 0
    cant_his = 0
    
    ser.flushInput()
    ser.flushOutput()
    ser.read_until(b' \r\n')

    #Ignora primero
    Leer_Sensor(ser)
    exit_ = False

    tiempo0 = time.time()
    while not exit_:
        hist = Leer_Sensor(ser)
        histy = np.array([int(i) for i in hist])

        if 'error' in hist:
            errr_cam = errr_cam + 1

        elif len(histy) == 256:
            ys = ys + histy
            cant_his = cant_his + 1

            if cant_his == args.c:
                np.save(f"{args.p}/{args.n}", ys)
                exit_ = True

        else:
            errr_ser = errr_ser + 1

        print(f"Histograma: {cant_his}\tError_Camara: {errr_cam}\tError_Serial: {errr_ser}\tCuentas_Totales: {np.sum(histy)}")
    
    tiempo1 = time.time()
    delta = tiempo1 - tiempo0
    anim.pause()

    if delta > 60:
        delta = delta/60
        print(f"Duracion de captura: {delta:.2f} min")
    else:
        print(f"Duracion de captura: {delta:.2f} seg")

    np.save(f"{args.p}/{args.n}", ys)
    plt.savefig(f"{args.p}/{args.n}.pdf")

    return tiempo1 - tiempo0

#In[2]: Procesamiento
def Animacion(i):
    global ys

    axs.clear()
    axs.plot(chns, ys, color='g')
    axs.set_xlim([xmin, xmax])
    axt.set_xlim([emin, emax])
    axs.set_yscale('log')

    axs.set_ylabel('Intensidad (a.u.)')
    axs.set_xlabel('Canales')
    axt.set_xlabel('Eneria (keV)')
    axs.grid(True)

    return None

if __name__ == "__main__":

    _thread.start_new_thread(Thread_Capturar, ())

    fig = plt.figure()
    axs = fig.add_subplot(1, 1, 1)
    axt = axs.twiny()

    axs.set_xlim([xmin, xmax])
    axt.set_xlim([emin, emax])
    axs.set_yscale('log')

    axs.set_ylabel('Intensidad (a.u.)')
    axs.set_xlabel('Canales')
    axt.set_xlabel('Eneria (keV)')
    axs.grid(True)

    anim = FuncAnimation(fig, Animacion, interval=1000)
    plt.show()

np.save(f"{args.p}/{args.n}", ys)
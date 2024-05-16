import serial

import numpy as np
import matplotlib.pyplot as plt

import pickle
import pandas as pd
from datetime import datetime

import time
from IPython import display

import ctypes
from picosdk.ps3000a import ps3000a as ps
from picosdk.functions import adc2mV, assert_pico_ok


### Velmex
velmex = serial.Serial(port='COM8', baudrate=9600, timeout=1)


def set_remote(on=True):
    mode = 'F' if on else 'Q'
    velmex.write(f'{mode}'.encode())
    print(velmex_get_responce())


def velmex_get_responce():
    responce = velmex.readline()
    return responce.decode().split('\r')


def velmex_print_responce():
    print('>>>')
    for line in velmex_get_responce():
        print(line)
    print('<<<')


def move(axis='x', d=10, direction=None):
    ''' Moves <axis> <d> mm increment in <direction> ('+' or '-').'''
    if direction is None:
        if d < 0:
            direction = '-'
        elif d >= 0:
            direction = '+'
    i = round(d * 200)  # i == number of steps
    assert axis in ['x', 'y', 'z']
    assert isinstance(d, (int, float))
    assert direction in ['+', '-']

    if direction == '+':
        direction = ''  # positive is implied by VELMEX
        if d == 0:
            return 'Zero move requested. Call move2max(<axis>) to roll axis to the positive limit.'

    if axis == 'x':
        cmd = f'(I4M{direction}{i},I1M{direction}{i},)'
    elif axis == 'y':
        cmd = f'I2M{direction}{i},'
    elif axis == 'z':
        cmd = f'I3M{direction}{i},'
    else:
        raise ValueError('wtf?')
    velmex.write(f'C {cmd}R'.encode())
    return velmex_get_responce()


def move2max(axis='x'):
    ''' Moves <axis> until the lower limit switch is hit.'''
    assert axis in ['x', 'y', 'z']
    if axis == 'x':
        cmd = f'(I4M0,I1M0,)'
    elif axis == 'y':
        cmd = f'I2M0,'
    elif axis == 'z':
        cmd = f'I3M0,'
    else:
        raise ValueError('wtf?')
    velmex.write(f'C {cmd}R'.encode())
    return velmex_get_responce()


def abs_move(axis, loc):
    ''' Move <axis> to the absolute position <loc> in mm.'''
    print(loc)
    # assert loc > 0

    pos = get_position(axis)

    dlt = loc - pos

    # print(dlt)

    if dlt == 0:
        return 'Already at location.'

    direction = '+' if dlt > 0 else '-'
    return move(axis=axis, d=dlt, direction=direction)


def velmex_wait_untill_ready(verbose=False):
    while True:
        velmex.write(b'V\r')
        status = velmex.readline().decode()
        if verbose:
            print(status, end=', ')
        if status == 'B':
            time.sleep(.3)
        else:
            flush = velmex_get_responce()
            break

    velmex.write(b'V\r')
    status = velmex.readline().decode()
    if status != 'R':
        raise ValueError(f'Unknown error: status={status}')
    return True


def home(axis='x'):
    ''' Moves <axis> until the lower limit switch is hit.
        Then zeros the motor's index.
    '''
    responce = move(axis, d=0, direction='-')
    if axis == 'x':
        cmd = f'(IA4M-0,IA1M-0,)'
    elif axis == 'y':
        cmd = f'IA2M-0,'
    elif axis == 'z':
        cmd = f'IA3M-0,'
    else:
        raise ValueError('wtf?')

    velmex_wait_untill_ready(verbose=False)

    velmex.write(f'C {cmd}R'.encode())
    responce += velmex_get_responce()
    return responce


def get_position(axis='x'):
    ''' Returns the absolute postion along <axis> in mm.'''
    assert axis in ['x', 'y', 'z']

    flush = velmex_get_responce()
    if axis == 'x':
        velmex.write(b'X T')
        i = np.mean(np.array((velmex_get_responce()[:-1]), dtype=int))
    else:
        velmex.write(axis.upper().encode())
        i = int(velmex_get_responce()[0])
    distance = i / 200.
    return distance


def is_master():
    ''' Returns 'True' if 2 VELMEXs are connected via bus and 'master' is this one
        (it must be set for remote operation).
    '''
    velmex.write(b'getD2\r')
    responce = velmex.readline()
    return responce.decode()[0] == '4'


def mulmove(a, b, c):
    velmex_wait_untill_ready()
    move(axis='x', d=abs(a), direction='+')
    velmex_wait_untill_ready()
    move(axis='y', d=abs(b), direction='-')
    velmex_wait_untill_ready()
    move(axis='z', d=abs(c), direction='+')
    return True


def reset():
    velmex_wait_untill_ready()
    home('x')
    velmex_wait_untill_ready()
    home('y')
    velmex_wait_untill_ready()
    home('z')
    return


set_remote(True)
is_master()


### Picoscope
class Picoscope():
    def __init__(self):
        # Create chandle and status ready for use
        self.status = {}
        self.chandle = ctypes.c_int16()

        # Opens the device/s
        self.status["openunit"] = ps.ps3000aOpenUnit(ctypes.byref(self.chandle), None)

        try:
            assert_pico_ok(self.status["openunit"])
        except:

            # powerstate becomes the status number of openunit
            powerstate = self.status["openunit"]

            # If powerstate is the same as 282 then it will run this if statement
            if powerstate == 282:
                # Changes the power input to "PICO_POWER_SUPPLY_NOT_CONNECTED"
                self.status["ChangePowerSource"] = ps.ps3000aChangePowerSource(self.chandle, 282)
                # If the powerstate is the same as 286 then it will run this if statement
            elif powerstate == 286:
                # Changes the power input to "PICO_USB3_0_DEVICE_NON_USB3_0_PORT"
                self.status["ChangePowerSource"] = ps.ps3000aChangePowerSource(self.chandle, 286)
            else:
                raise

            assert_pico_ok(self.status["ChangePowerSource"])

    def setChannel(self, chNumber, chRange):
        self.chRange = chRange
        if chNumber == 0:
            self.status["setChA"] = ps.ps3000aSetChannel(self.chandle, 0, 1, 1, chRange, 0)
            assert_pico_ok(self.status["setChA"])
        elif chNumber == 1:
            self.status["setChB"] = ps.ps3000aSetChannel(self.chandle, 1, 1, 1, chRange, 0)
            assert_pico_ok(self.status["setChB"])
        elif chNumber == 2:
            self.status["setChC"] = ps.ps3000aSetChannel(self.chandle, 2, 1, 1, chRange, 0)
            assert_pico_ok(self.status["setChC"])
        elif chNumber == 3:
            self.status["setChD"] = ps.ps3000aSetChannel(self.chandle, 3, 1, 1, chRange, 0)
            assert_pico_ok(self.status["setChD"])
        else:
            raise Exception("This Picoscope only has 4 channels my friend")

    def setTrigger(self):
        self.status["trigger"] = ps.ps3000aSetSimpleTrigger(self.chandle, 1, 0, 1024, 3, 0, 1000)
        assert_pico_ok(self.status["trigger"])

    # def setBuffer(self):

    def setMeasurementSettings(self, preTriggerSamples, postTriggerSamples, Timebase):
        self.preTriggerSamples = preTriggerSamples
        self.postTriggerSamples = postTriggerSamples
        self.maxsamples = preTriggerSamples + postTriggerSamples
        self.Timebase = Timebase

    def Measure(self):
        overflow = ctypes.c_int16()
        cmaxSamples = ctypes.c_int32(self.maxsamples)

        self.status["runblock"] = ps.ps3000aRunBlock(self.chandle, self.preTriggerSamples, self.postTriggerSamples,
                                                     self.Timebase, 1, None, 0, None, None)
        assert_pico_ok(self.status["runblock"])

        bufferAMax = (ctypes.c_int16 * self.maxsamples)()
        bufferAMin = (ctypes.c_int16 * self.maxsamples)()

        bufferBMax = (ctypes.c_int16 * self.maxsamples)()
        bufferBMin = (ctypes.c_int16 * self.maxsamples)()

        self.bufferAMax = bufferAMax
        self.bufferAMin = bufferAMin

        self.bufferBMax = bufferBMax
        self.bufferBMin = bufferBMin

        self.status["SetDataBuffersA"] = ps.ps3000aSetDataBuffers(self.chandle, 0, ctypes.byref(bufferAMax),
                                                                  ctypes.byref(bufferAMin), self.maxsamples, 0, 0)
        assert_pico_ok(self.status["SetDataBuffersA"])

        self.status["SetDataBuffersB"] = ps.ps3000aSetDataBuffers(self.chandle, 1, ctypes.byref(bufferBMax),
                                                                  ctypes.byref(bufferBMin), self.maxsamples, 0, 0)
        assert_pico_ok(self.status["SetDataBuffersB"])

        # Creates a overlow location for data
        overflow = (ctypes.c_int16 * 10)()
        # Creates converted types maxsamples
        cmaxSamples = ctypes.c_int32(self.maxsamples)
        self.cmaxSamples = cmaxSamples

        # Checks data collection to finish the capture
        ready = ctypes.c_int16(0)
        check = ctypes.c_int16(0)
        while ready.value == check.value:
            self.status["isReady"] = ps.ps3000aIsReady(self.chandle, ctypes.byref(ready))

        self.status["GetValues"] = ps.ps3000aGetValues(self.chandle, 0, ctypes.byref(cmaxSamples), 0, 0, 0,
                                                       ctypes.byref(overflow))
        assert_pico_ok(self.status["GetValues"])

    def plotMeasurement(self):
        maxADC = ctypes.c_int16()
        self.status["maximumValue"] = ps.ps3000aMaximumValue(self.chandle, ctypes.byref(maxADC))
        assert_pico_ok(self.status["maximumValue"])

        # Converts ADC from channel A to mV
        adc2mVChAMax = adc2mV(self.bufferAMax, self.chRange, maxADC)

        # Converts ADC from channel B to mV
        adc2mVChBMax = adc2mV(self.bufferBMax, self.chRange, maxADC)

        timeIntervalns = ctypes.c_float()
        returnedMaxSamples = ctypes.c_int16()
        self.status["GetTimebase"] = ps.ps3000aGetTimebase2(self.chandle, self.Timebase, self.maxsamples,
                                                            ctypes.byref(timeIntervalns), 1,
                                                            ctypes.byref(returnedMaxSamples), 0)
        assert_pico_ok(self.status["GetTimebase"])

        # Creates the time data
        time = np.linspace(0, (self.cmaxSamples.value - 1) * timeIntervalns.value, self.cmaxSamples.value)

        plt.plot(1e-3 * time, adc2mVChAMax[:], '.', label='Sent')
        plt.plot(1e-3 * time, adc2mVChBMax[:], '.', label='Received')
        plt.xlabel('Time (us)')
        plt.ylabel('Voltage (mV)')
        plt.title(datetime.now().strftime("%d/%m/%Y %H:%M:%S"))
        plt.legend()
        plt.grid()
        plt.show()

    def getMetrics(self):
        maxADC = ctypes.c_int16()
        self.status["maximumValue"] = ps.ps3000aMaximumValue(self.chandle, ctypes.byref(maxADC))
        assert_pico_ok(self.status["maximumValue"])

        # Converts ADC from channel A to mV
        adc2mVChAMax = adc2mV(self.bufferAMax, self.chRange, maxADC)

        # Converts ADC from channel B to mV
        adc2mVChBMax = adc2mV(self.bufferAMax, self.chRange, maxADC)

        p2pA = np.max(adc2mVChAMax) - np.min(adc2mVChAMax)
        p2pB = np.max(adc2mVChBMax) - np.min(adc2mVChBMax)

        return p2pA, p2pB

    def getMeasurementData(self):
        maxADC = ctypes.c_int16()
        self.status["maximumValue"] = ps.ps3000aMaximumValue(self.chandle, ctypes.byref(maxADC))
        assert_pico_ok(self.status["maximumValue"])

        # Converts ADC from channel A to mV
        adc2mVChAMax = adc2mV(self.bufferAMax, self.chRange, maxADC)

        # Converts ADC from channel B to mV
        adc2mVChBMax = adc2mV(self.bufferBMax, self.chRange, maxADC)

        timeIntervalns = ctypes.c_float()
        returnedMaxSamples = ctypes.c_int16()
        self.status["GetTimebase"] = ps.ps3000aGetTimebase2(self.chandle, self.Timebase, self.maxsamples,
                                                            ctypes.byref(timeIntervalns), 1,
                                                            ctypes.byref(returnedMaxSamples), 0)
        assert_pico_ok(self.status["GetTimebase"])

        # Creates the time data
        time = np.linspace(0, (self.cmaxSamples.value - 1) * timeIntervalns.value, self.cmaxSamples.value)

        return time, adc2mVChAMax, adc2mVChBMax

    def StopandClose(self):
        # Stops the scope
        self.status["stop"] = ps.ps3000aStop(self.chandle)
        assert_pico_ok(self.status["stop"])

        # Closes the unit
        self.status["close"] = ps.ps3000aCloseUnit(self.chandle)
        assert_pico_ok(self.status["close"])

    def returnStatus(self):
        return self.status


pico = Picoscope()


### Saving data
def save_data(filename, name, frequency, amplitude, location, values, extra=None):
    dictionary = {'date': datetime.now().strftime("%d/%m/%Y %H:%M:%S"),
                  'frequency[Hz]': frequency,
                  'Amplitude[V]': amplitude,
                  'location': location,
                  'values': values,
                  'extra': extra}

    try:
        with open(f"{filename}.pkl", 'rb') as file:
            df = pickle.load(file)

        while name in df.keys():
            name = input('Name in use, Other name:')
        df[name] = dictionary

        with open(f"{filename}.pkl", 'wb') as file:
            pickle.dump(df, file)

    except:
        df = pd.DataFrame()
        df[name] = dictionary

        with open(f"{filename}.pkl", 'wb') as file:
            pickle.dump(df, file)

        print(f"{filename}.pkl is created")
    print(name, ' successfully saved in: ', filename)
    return


### Measuring
def measure_now(channel=[2, 1], settings=[2000, 8000, 2]):
    pico.setChannel(0, channel[0])  # wat we uitzenden
    pico.setChannel(1, channel[1])  # wat we verkrijgen
    pico.setTrigger()
    pico.setMeasurementSettings(settings[0], settings[1], settings[2])
    pico.Measure()
    plt.figure(figsize=(10, 6))
    plt.title('1 KHz Modulating Wave - 500 KHz Carrier Wave')
    pico.plotMeasurement()
    return


def measure_1D(tot_dis, dx=None, samples=None, axis='x', direction='+', channel=[2, 1],
               settings=[2000, 800, 2], plot=True, return_to_start=True):
    if dx and samples:
        if tot_dis / samples != dx:
            print('Samples and dx do not max the total distance!')
        return
    if dx and not samples:
        samples = int(tot_dis / dx)
        print('tot_dis=', tot_dis, 'dx=', dx, 'samples = ', samples, '\n')
    if not dx and samples:
        dx = tot_dis / samples
        print('tot_dis=', tot_dis, 'dx=', dx, 'samples = ', samples, '\n')

    if direction == '+':
        dir1, dir2 = '+', '-'
    if direction == '-':
        dir1, dir2 = '-', '+'

    max_values = []

    for i in range(samples):
        pico.setChannel(0, channel[0])  # wat we uitzenden
        pico.setChannel(1, channel[1])  # wat we verkrijgen
        pico.setTrigger()
        pico.setMeasurementSettings(settings[0], settings[1], settings[2])
        pico.Measure()
        time, p1, p2 = pico.getMeasurementData()
        max_values.append([p1, p2])

        move(axis=axis, d=tot_dis / samples, direction=dir1)  # altijd 1 extra sample!!!
        velmex_wait_untill_ready()

        if plot:
            pico.plotMeasurement()

        display.display(plt.gcf())
        display.clear_output(wait=True)

        plt.clf()

    if return_to_start:
        move(axis=axis, d=tot_dis, direction=dir2)

    return max_values


def measure_2D_xy(filename, name, frequency, amplitude, location, extra,
                  x_tot, y_tot, dx=None, dy=None, samples_x=None, samples_y=None,
                  plot=True, channel=[2, 1]):
    if dy and not samples_y:
        samples_y = int(y_tot / dy)

    if not dy and samples_y:
        dy = y_tot / samples_y

    all_values = []

    move(axis='y', d=y_tot / 2, direction='+')
    velmex_wait_untill_ready()
    Last = samples_y % 2 == 0
    for i in range(samples_y):
        print('\r', 'count: ' + str(round(i / samples_y * 100)) + '%      ', end='')
        if i % 2 == 0:
            values = measure_1D(tot_dis=x_tot,
                                dx=dx,
                                samples=samples_x,
                                direction='+',
                                return_to_start=False, channel=[2, 1])
            end_direction = '-'

        if i % 2 == 1:
            values = measure_1D(tot_dis=x_tot,
                                dx=dx,
                                samples=samples_x,
                                direction='-',
                                return_to_start=False, channel=[2, 1])[::-1]
            end_direction = '+'

        move(axis='y', d=dy, direction='-')
        velmex_wait_untill_ready()

        if i == samples_y - 1:
            values = measure_1D(tot_dis=x_tot,
                                dx=dx,
                                samples=samples_x,
                                direction=end_direction,
                                return_to_start=False)[::-1]

        all_values.append(values)

        save_data(filename=filename,
                  name=name + str(i),
                  frequency=frequency,
                  amplitude=amplitude,
                  location=i,
                  values=values,
                  extra=extra)

        save_data(filename=filename + '_all',
                  name=name + '_all_' + str(i),
                  frequency=frequency,
                  amplitude=amplitude,
                  location=i,
                  values=all_values,
                  extra=extra)

    move(axis='y', d=y_tot / 2, direction='+')
    velmex_wait_untill_ready()
    move(axis='x', d=x_tot, direction='-')
    velmex_wait_untill_ready()

    print('ready')

    return all_values


### Find maximum functions
def findmax(m, tot_dis):
    data = m
    maxs_ = []
    length = len(data)
    xw = [i * tot_dis / length for i in range(length)]
    for j in range(np.shape(m)[0]):
        send, measured = data[j][0], data[j][1]
        measured_max = np.sqrt(2 * np.sum(np.array(measured) ** 2 / len(measured)))
        maxs_.append(measured_max)

        plt.plot(measured, 'b', label='hydrofoon')
        plt.plot(np.ones(len(measured)) * measured_max, 'y--')
        #     plt.title(f'{row, col}')
        plt.legend()

        #     plt.show()
        display.display(plt.gcf())
        display.clear_output(wait=True)

        plt.clf()
    plt.plot(xw, maxs_)
    plt.show()
    return maxs_, xw


def find_max_ampl_1D(axis, tot_dist, samples):
    print('Move', axis, tot_dist)
    move(axis=axis, d=tot_dist / 2, direction='-')
    values = measure_1D(tot_dis=tot_dist,
                        dx=None, samples=samples,
                        axis=axis, direction='+',
                        channel=[2, 1], settings=[2000, 8000, 2],
                        plot=True, return_to_start=True)
    maxs, xw = findmax(values, tot_dis=tot_dist)
    go_to = xw[maxs.index(max(maxs))]
    print('Move', axis, go_to, '?')
    confirm = str(input())
    if confirm != str(1):
        print('\nNOT Confirmed!')
        return
    move(axis=axis, d=go_to, direction='+')
    velmex_wait_untill_ready()
    plt.plot(xw, maxs)
    return


def find_max_ampl_3D(tot_dist, samples):
    for axis in ['x', 'y', 'z']:
        print(axis)
        print('Move', axis, tot_dist)
        confirm = str(input())
        if confirm == str(1):
            print('Confirmed!')
            move(axis=axis, d=tot_dist / 2, direction='-')
            values = measure_1D(tot_dis=tot_dist,
                                dx=None, samples=samples,
                                axis=axis, direction='+',
                                channel=[2, 1], settings=[2000, 8000, 2],
                                plot=True, return_to_start=True)
            maxs, xw = findmax(values, tot_dis=tot_dist)
            go_to = xw[maxs.index(max(maxs))]
            print('Move', axis, go_to, '?')
            confirm = str(input())
            if confirm != str(1):
                print('\nNOT Confirmed!')
                break
            move(axis=axis, d=go_to, direction='+')
            velmex_wait_untill_ready()
            plt.plot(xw, maxs)
        else:
            print('\nNOT Confirmed!')
            break
    return


### Calibration
def detect_blocks(signal, threshold):
    blocks = []
    block_started = False

    for index, value in enumerate(signal):
        if value >= threshold and not block_started:
            blocks.append(index)
            block_started = True
        elif value < threshold and block_started:
            block_started = False

    return np.array(blocks)

def time_of_flight(sample_size):
    times = []

    for _ in range(sample_size):
        pico.setChannel(0, 2) #wat we uitzenden
        pico.setChannel(1, 1) #wat we verkrijgen
        pico.setTrigger()
        pico.setMeasurementSettings(2000, 800000, 2)
        pico.Measure()

        _, a, b = pico.getMeasurementData()

        sent = a  # Placeholder for sent signal data
        sent = np.convolve(np.abs(sent), np.ones(1000), 'valid') / 1000
        received = b  # Placeholder for received signal data
        received = np.convolve(np.abs(received), np.ones(1000), 'valid') / 1000

        # Plotting for visualization, replace with actual plotting functions
        # Assuming len(sent) is the number of samples
        num_samples = len(sent)

        # Create the time axis
        time_axis = np.arange(0, num_samples * 4 * 10**-9, 4 * 10**-9)

        # Plot the sent signal against the time axis
        plt.plot(time_axis, sent, label='sent')
        plt.plot(time_axis, received, label='received')
        plt.legend(loc='upper right')
        plt.show()

        sent_blocks = detect_blocks(sent, np.mean(sent))
        received_blocks = detect_blocks(received, np.mean(received))

        print("Sent Blocks (samples):", sent_blocks * 4 *10**-9)
        print('Received Blocks (samples):', received_blocks * 4 * 10**-9)

        # Calculating time of flight and storing in times list

        if sent_blocks[0] == 0 and received_blocks[0] == 0:

            sent_blocks = sent_blocks[1:]
            received_blocks = received_blocks[2:]

            print(sent_blocks* 4 * 10**-9, received_blocks* 4 * 10**-9)

            if len(sent_blocks) >= len(received_blocks):
                sent_blocks = sent_blocks[:-(len(sent_blocks)-len(received_blocks))]
            elif len(sent_blocks) < len(received_blocks):
                received_blocks = received_blocks[:-(len(received_blocks)-len(sent_blocks))]

            differences = received_blocks - sent_blocks

        elif sent_blocks[0] == 0 and received_blocks[0] != 0:

            sent_blocks = sent_blocks[1:]
            received_blocks = received_blocks[1:]

            print(sent_blocks* 4 * 10**-9, received_blocks* 4 * 10**-9)

            if len(sent_blocks) > len(received_blocks):
                sent_blocks = sent_blocks[:-(len(sent_blocks)-len(received_blocks))]

            elif len(sent_blocks) < len(received_blocks):
                received_blocks = received_blocks[:-(len(received_blocks)-len(sent_blocks))]

            differences = received_blocks - sent_blocks

        elif sent_blocks[0] != 0 and received_blocks[0] == 0:

            sent_blocks = sent_blocks[:]
            received_blocks = received_blocks[1:]

            print(sent_blocks* 4 * 10**-9, received_blocks* 4 * 10**-9)

            if len(sent_blocks) >= len(received_blocks):
                sent_blocks = sent_blocks[:-(len(sent_blocks)-len(received_blocks))]
            elif len(sent_blocks) < len(received_blocks):
                received_blocks = received_blocks[:-(len(received_blocks)-len(sent_blocks))]

            differences = received_blocks - sent_blocks

        else:

            sent_blocks = sent_blocks[:]
            received_blocks = received_blocks[:]

            print(sent_blocks* 4 * 10**-9, received_blocks* 4 * 10**-9)

            if len(sent_blocks) >= len(received_blocks):
                sent_blocks = sent_blocks[:-(len(sent_blocks)-len(received_blocks))]
            elif len(sent_blocks) < len(received_blocks):
                received_blocks = received_blocks[:-(len(received_blocks)-len(sent_blocks))]

            differences = received_blocks - sent_blocks
        print(differences * 4 * 10**-9)
        times.extend(differences)

    return times


def calibration(sample_size, ref_distance):
    times = time_of_flight(sample_size)

    if times:

        Channel_delay = 0.0006269698039215688  # mean delay on the channels of the signal generator in seconds
        mean_tof = np.mean(times) * 4 * 10 ** -9 - Channel_delay  # each sample is 4 ns
        std_tof = np.std(times) * 4 * 10 ** -9  # each sample is 4 ns
        tof_ref = ref_distance / 1500  # Reference time of flight

        print('Sample Size:', len(times), 'samples.')
        print('Mean Time of Flight (s):', mean_tof)
        print('Standard Deviation of Time of Flight (s):', std_tof)
        print('Time of Flight Reference (s):', tof_ref)

        speed = 1500  # m/s

        delta_tof = mean_tof - tof_ref  # s
        distance = delta_tof * speed

        print('Difference between reference point and current point (s):', delta_tof)
        print('Distance to be moved (m):', distance)

        # Move function call based on distance
        if distance <= 0:
            print("Move function call for negative distance.")
        else:
            print("Move function call for positive distance.")
        return distance, mean_tof * speed
    else:
        print("No data collected for calibration.")

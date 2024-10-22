####################################################
# 
# This sample script shows how to connect to ARIADNE
# via BLE and trigger alternating vibrations to the
# two actuator modules, once per second.
#
# Authors: Julian Martus and Nataliya Rokhmanova
#
####################################################

import asyncio
from bleak import BleakScanner, BleakClient
import time
import struct
import datetime

BLE_DURATION_STIM_SERVICE_UUID = '1111'
BLE_AMPLITUDE_CHARACTERISTIC_UUID = '1112'  
BLE_DURATION_B_CHARACTERISTIC_UUID = '1113'  
BLE_DURATION_A_CHARACTERISTIC_UUID = '1114' 

BLE_BATTERY_SERVICE_UUID = '180F'
BLE_BATTERY_LEVEL_CHARACTERISTIC_UUID = '2A19'

timeout = 5     # ARIADNE will attempt to connect for five seconds

async def connect_to_device():
    devices = await BleakScanner.discover()
    for d in devices:
        if d.name == 'Ariadne':
            print('Device found - MAC [', d.address, ']')
            client = BleakClient(d.address)
            await client.connect(timeout=timeout)
            print('Connected [', d.address, ']')
            return client

async def get_characteristic(service, characteristic_uuid):
    characteristic = service.get_characteristic(characteristic_uuid)
    return characteristic

async def write_characteristic(client, characteristic, value):
    value_bytes = struct.pack('<H', value)
    await client.write_gatt_char(characteristic, value_bytes)

async def read_characteristic(client, characteristic):
    value = await client.read_gatt_char(characteristic)
    return value

async def set_amp(client, characteristic, value):
    await client.write_gatt_char(characteristic,  bytearray([value]))


async def run():
    
    Ariadne = await connect_to_device()
    LRA_service = Ariadne.services.get_service(BLE_DURATION_STIM_SERVICE_UUID)
    BAT_service = Ariadne.services.get_service(BLE_BATTERY_SERVICE_UUID)

    if LRA_service:
        DeviceB_dur = await get_characteristic(LRA_service, BLE_DURATION_B_CHARACTERISTIC_UUID)
        DeviceA_dur = await get_characteristic(LRA_service, BLE_DURATION_A_CHARACTERISTIC_UUID)
        Ampl = await get_characteristic(LRA_service, BLE_AMPLITUDE_CHARACTERISTIC_UUID)
    else:
        print("Ariadne was not found. Make sure the device is turned on and the blue LED is pulsing slowly.")
        exit()
    if BAT_service:    
        Bat = await get_characteristic(BAT_service, BLE_BATTERY_LEVEL_CHARACTERISTIC_UUID)
    else:
        print("Battery level not found - reading is ignored")

    await set_amp(Ariadne, Ampl, 127)
    count = 0
    while (Ariadne.is_connected):
        await write_characteristic(Ariadne, DeviceB_dur, 120)       # trigger a 120ms vibration 
        time.sleep(1)       # Sleep for 1 second


        await write_characteristic(Ariadne, DeviceA_dur, 120)       # trigger a 120ms vibration 
        time.sleep(1)       # Sleep for 1 second

        if Bat: 
            batteryLevel = await read_characteristic(Ariadne, Bat)
            batteryLevel_int = int.from_bytes(batteryLevel, "little")

        current_time = datetime.datetime.now()  # get the current date and time

        print(f'{current_time}: Bat_Level = [{batteryLevel_int}%]')

        count = count +1

    await Ariadne.disconnect()
    
loop = asyncio.get_event_loop()
loop.run_until_complete(run())
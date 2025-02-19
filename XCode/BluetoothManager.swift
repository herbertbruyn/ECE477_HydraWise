//
//  BluetoothManager.swift
//  HydraWise
//
//  Created by James Neff on 2/6/25.
//

import CoreBluetooth
import SwiftUI

class BluetoothManager: NSObject, ObservableObject, CBCentralManagerDelegate, CBPeripheralDelegate {
    var centralManager: CBCentralManager!
    var peripheralDevice: CBPeripheral?
    
    // Service UUIDS
    let heartRateServiceUUID = CBUUID(string: "0000180D-0000-1000-8000-00805F9B34FB") // Placeholder UUID
    let hydrationServiceUUID = CBUUID(string: "12345678-1234-5678-1234-567812345678") // Placeholder UUID

    // Characteristic UUIDS
    let heartRateCharacteristicUUID = CBUUID(string: "00002A37-0000-1000-8000-00805F9B34FB") // Placeholder UUID
    let hydrationCharacteristicUUID = CBUUID(string: "87654321-4321-8765-4321-876543218765") // Placeholder UUID

    // Store hydration characteristic
    var hydrationCharacteristic: CBCharacteristic?
    
    // @Published makes variables observable, so SwiftUI views update instantly
    @Published var heartRate: Int = 0
    @Published var hydrationLevel: Float = 0.0
    
    // Create BLE manager with itself as delegate
    override init() {
        super.init()
        centralManager = CBCentralManager(delegate: self, queue: nil)
    }

    // Scans for devices with correct services - ensures bluetooth is on prior to scanning
    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        if central.state == .poweredOn {
            print("Bluetooth ON, scanning for devices...")
            centralManager.scanForPeripherals(withServices: [heartRateServiceUUID, hydrationServiceUUID], options: nil)
        } else {
            print("Bluetooth is OFF.")
        }
    }
    
    // Stops scanning if correct device found - saves reference to connected device
    func centralManager(_ central: CBCentralManager, didDiscover peripheral: CBPeripheral, advertisementData: [String : Any], rssi RSSI: NSNumber) {
        print("Discovered device: \(peripheral.name ?? "Unknown")")

        peripheralDevice = peripheral
        peripheralDevice?.delegate = self

        centralManager.stopScan()
        centralManager.connect(peripheral, options: nil)
    }

    // Search for services on device
    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
        print("Connected to: \(peripheral.name ?? "Unknown")")
        peripheral.discoverServices([heartRateServiceUUID, hydrationServiceUUID])
    }
    
    // Identify the services - search for characteristics inside each service
    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
        if let services = peripheral.services {
            for service in services {
                if service.uuid == heartRateServiceUUID {
                    peripheral.discoverCharacteristics([heartRateCharacteristicUUID], for: service)
                }
                if service.uuid == hydrationCharacteristicUUID {
                    peripheral.discoverCharacteristics([hydrationCharacteristicUUID], for: service)
                }
            }
        }
    }
    
    // Pushes data automically
    func peripheral(_ peripheral: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) {
        if let characteristics = service.characteristics {
            for characteristic in characteristics {
                if characteristic.uuid == heartRateCharacteristicUUID {
                    peripheral.setNotifyValue(true, for: characteristic)
                }
                if characteristic.uuid == hydrationCharacteristicUUID {
                    peripheral.setNotifyValue(true, for: characteristic)
                    hydrationCharacteristic = characteristic
                }
            }
        }
    }
    
    // Read data from sensor
    func peripheral(_ peripheral: CBPeripheral, didUpdateValueFor characteristic: CBCharacteristic, error: Error?) {
        if let data = characteristic.value {
            if characteristic.uuid == heartRateCharacteristicUUID {
                heartRate = extractHeartRate(from: data)
                print("Heart Rate: \(heartRate) BPM")
            }
            if characteristic.uuid == hydrationCharacteristicUUID {
                hydrationLevel = extractHydrationLevel(from: data)
                print("Hydration Level: \(hydrationLevel)%")
            }
        }
    }
    
    // Get heartrate
    func extractHeartRate(from data: Data) -> Int {
        var bpm: UInt8 = 0
        data.copyBytes(to: &bpm, count: 1)
        return Int(bpm)
    }

    // Get hydration level
    func extractHydrationLevel(from data: Data) -> Float {
        var level: Float = 0
        (data as NSData).getBytes(&level, length: MemoryLayout<Float>.size)
        return level
    }
    
    // Auto reconnect if device disconnects
    func centralManager(_ central: CBCentralManager, didDisconnectPeripheral peripheral: CBPeripheral, error: Error?) {
        print("Device disconnected. Reconnecting...")
        centralManager.connect(peripheral, options: nil)
    }

    // When button is pressed to start hydration reading
    func sendHydrationCommand() {
        guard let peripheral = peripheralDevice, let characteristic = hydrationCharacteristic else {
            print("ESP32 not connected or characteristic not found!")
            return
        }
        
        let command = "START".data(using: .utf8)!
        peripheral.writeValue(command, for: characteristic, type: .withResponse)
        print("Sent 'START' command to ESP32")
    }


}

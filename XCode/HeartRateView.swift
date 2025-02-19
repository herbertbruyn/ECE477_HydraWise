//
//  HeartRateView.swift
//  HydraWise
//
//  Created by James Neff on 2/4/25.
//

import SwiftUI

struct HeartRateView: View {
    // Receive val from bluetooth manager
    @ObservedObject var bluetoothManager: BluetoothManager
    
    var body: some View {
        VStack {
            Text("Heart Rate")
                .font(.title)
                .padding()
            
            Text("\(bluetoothManager.heartRate) BPM")
                .font(.largeTitle)
                .bold()
                .foregroundColor(.red)
                .padding()
            
            Circle()
                .strokeBorder(lineWidth: 24)
                foregroundColor(.red)
        }
    }
}

struct HeartRateView_Previews: PreviewProvider {
    static var previews: some View {
            let mockBluetoothManager = BluetoothManager()
            mockBluetoothManager.heartRate = 75 // Fake data for preview
            return HeartRateView(bluetoothManager: mockBluetoothManager)
    }
}


//
//  HydrationView.swift
//  HydraWise
//
//  Created by James Neff on 2/4/25.
//

import SwiftUI

struct HydrationView: View {
    @ObservedObject var bluetoothManager: BluetoothManager

    var body: some View {
        List {
            Section(header: Text("Sweat Information")) {
                Button(action: {
                    bluetoothManager.sendHydrationCommand()
                }) {
                    Label("Start Reading", systemImage: "timer")
                        .font(.headline)
                        .foregroundColor(.accentColor)
                }
                HStack {
                    Label("Water Concetration", systemImage: "clock")
                    Spacer()
                    Text("\(bluetoothManager.hydrationLevel) %")
                }
                .accessibilityElement(children: .combine)
                HStack {
                    Label("Recommendation", systemImage: "paintpalette")
                    Spacer()
                    Text("Water")
                        .padding(4)
                        .cornerRadius(4)
                }
                .accessibilityElement(children: .combine)
            }
        }
    }
}

struct HydrationView_Previews: PreviewProvider {
    static var previews: some View {
        let mockBluetoothManager = BluetoothManager()
        mockBluetoothManager.hydrationLevel = 85.0 // Fake hydration value

        return HydrationView(bluetoothManager: mockBluetoothManager)
    }
}

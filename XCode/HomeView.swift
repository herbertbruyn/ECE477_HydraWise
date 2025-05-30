//
//  HomeView.swift
//  HydraWise
//
//  Created by James Neff on 2/4/25.
//

import SwiftUI

struct HomeView: View{
    // Bluetooth instance
    @StateObject private var bluetoothManager = BluetoothManager()
    
    var body: some View {
        NavigationStack {
            NavigationLink(destination: HeartRateView(bluetoothManager: bluetoothManager)) {
                    BarView(title: "Heart Rate", color: .red, icon: "heart.fill")
                }
            NavigationLink(destination: HydrationView(bluetoothManager: bluetoothManager)) {
                    BarView(title: "Hydration Levels", color: .blue, icon: "drop.fill")
                }
            .navigationTitle("Health Data")
            Spacer()
        }
    }
}

struct HomeView_Previews: PreviewProvider {
    static var previews: some View {
        HomeView()
    }
}

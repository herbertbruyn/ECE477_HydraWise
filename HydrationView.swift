//
//  HydrationView.swift
//  HydraWise
//
//  Created by James Neff on 2/4/25.
//

import SwiftUI

struct HydrationView: View {
    let stat : HealthMetrics
    
    var body: some View {
        List {
            Section(header: Text("Sweat Information")) {
                Label("Start Reading", systemImage: "timer")
                    .font(.headline)
                    .foregroundColor(.accentColor)
                HStack {
                    Label("Water Concetration", systemImage: "clock")
                    Spacer()
                    Text("92 %")
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
        NavigationStack {
            HydrationView(stat: HealthMetrics.sampleData[0])
        }
    }
}

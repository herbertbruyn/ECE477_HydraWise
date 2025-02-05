//
//  BarView.swift
//  HydraWise
//
//  Created by James Neff on 2/4/25.
//

import SwiftUI

struct BarView: View {
    let title: String
    let color: Color
    let icon: String

    var body: some View {
        HStack {
            Image(systemName: icon)
                .font(.title)
                .foregroundColor(.white)
            
            Text(title)
                .font(.headline)
                .foregroundColor(.white)
            
            Spacer()
        }
        .padding()
        .frame(maxWidth: .infinity)
        .background(color)
        .cornerRadius(10)
        .shadow(radius: 3)
        .padding(.horizontal)
    }
}

// First Page - Heart Rate
struct HeartRatePage: View {
    var body: some View {
        Text("Heart Rate Monitoring")
            .font(.title)
            .padding()
    }
}

// Second Page - Hydration Levels
struct HydrationPage: View {
    var body: some View {
        Text("Hydration Tracking")
            .font(.title)
            .padding()
    }
}


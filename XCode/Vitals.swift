//
//  Vitals.swift
//  HydraWise
//
//  Created by James Neff on 2/4/25.
//

import Foundation

struct HealthMetrics: Identifiable {
    let id: UUID
    var heartrate: String
    var hydration: String
    
    init(id: UUID = UUID(), heartrate: String, hydration: String) {
        self.id = id
        self.heartrate = heartrate
        self.hydration = hydration
    }
}

extension HealthMetrics {
    struct Attendee: Identifiable {
        let id: UUID
        var name: String
        
        init(id: UUID = UUID(), name: String) {
            self.id = id
            self.name = name
        }
    }
    
}

extension HealthMetrics {
    static let sampleData: [HealthMetrics] =
    [
        HealthMetrics(heartrate: "62", hydration: "92 %")
    ]
}

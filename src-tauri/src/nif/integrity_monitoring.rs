//! Integrity Monitoring Module
//! Ensures navigation system reliability and fault detection

use std::collections::HashMap;
use serde::{Deserialize, Serialize};
use ndarray::{Array2, Array1};

/// System health assessment
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SystemHealth {
    pub overall_health: f64,
    pub subsystem_health: HashMap<String, f64>,
    pub issues: Vec<String>,
    pub timestamp: f64,
}

/// Integrity monitoring system
pub struct IntegrityMonitor {
    pub monitors: HashMap<String, Box<dyn IntegrityCheck>>,
    pub alert_thresholds: HashMap<String, f64>,
    pub fault_history: Vec<FaultEvent>,
    pub system_health: SystemHealth,
}

/// Integrity check interface
#[async_trait::async_trait]
pub trait IntegrityCheck: Send + Sync {
    /// Perform integrity check
    async fn check_integrity(&self, data: &SensorData) -> Result<IntegrityResult, String>;

    /// Get check name
    fn name(&self) -> &str;
}

/// Integrity check result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IntegrityResult {
    pub is_integrity_maintained: bool,
    pub confidence_level: f64,
    pub detected_faults: Vec<FaultType>,
    pub protection_level: f64,
}



/// Fault event record
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FaultEvent {
    pub timestamp: f64,
    pub fault_type: FaultType,
    pub severity: FaultSeverity,
    pub affected_subsystems: Vec<String>,
    pub mitigation_action: String,
}

/// Fault types
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum FaultType {
    SensorFailure(String),
    CommunicationLoss(String),
    ComputationError(String),
    EnvironmentalDisturbance(String),
    CyberAttack(String),
    FalseIntegrityAlarm,
}

/// Fault severity levels
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum FaultSeverity {
    Minor,
    Moderate,
    Critical,
    Catastrophic,
}

/// RAIM (Receiver Autonomous Integrity Monitoring) implementation
pub struct RAIMMonitor {
    pub test_statistics: Vec<f64>,
    pub fault_detection_threshold: f64,
    pub integrity_risk: f64,
    pub protection_level: f64,
}

impl RAIMMonitor {
    pub fn new() -> Self {
        RAIMMonitor {
            test_statistics: Vec::new(),
            fault_detection_threshold: 5.0, // Chi-squared threshold
            integrity_risk: 1e-7, // Target integrity risk
            protection_level: 0.0,
        }
    }
}

/// IntegrityMonitor: high-level monitor aggregating various subsystems
/// (This impl block adds the monitor_system async method to the IntegrityMonitor type)
impl IntegrityMonitor {
    /// Monitor system integrity
    pub async fn monitor_system(&mut self, sensor_data: &SensorData) -> SystemHealth {
        let mut health_score = 1.0;
        let mut issues = Vec::new();

        // RAIM monitoring
        if let Some(raim_result) = self.raim_monitor.monitor(sensor_data).await {
            if !raim_result.is_integrity_ok {
                health_score *= 0.8;
                issues.push("RAIM integrity violation detected".to_string());
            }
        }

        // FDE monitoring
        if let Some(fde_result) = self.fde_monitor.monitor(sensor_data).await {
            if !fde_result.is_integrity_ok {
                health_score *= 0.7;
                issues.push("FDE integrity violation detected".to_string());
            }
        }

        // Multi-constellation monitoring
        if let Some(multi_result) = self.multi_constellation_monitor.monitor(sensor_data).await {
            if !multi_result.is_integrity_ok {
                health_score *= 0.9;
                issues.push("Multi-constellation integrity issue".to_string());
            }
        }

        // Anomaly detection
        if let Some(anomaly_result) = self.anomaly_detector.detect(sensor_data).await {
            if anomaly_result.is_anomaly {
                health_score *= 0.85;
                issues.push(format!("Anomaly detected: {}", anomaly_result.description));
            }
        }

        SystemHealth {
            overall_health: health_score,
            subsystem_health: HashMap::new(), // Placeholder
            issues,
            timestamp: sensor_data.timestamp,
        }
    }

    /// Perform RAIM check on pseudorange measurements
    pub fn check_pseudoranges(&mut self, measurements: &[f64]) -> IntegrityResult {
        if measurements.len() < 5 {
            return IntegrityResult {
                is_integrity_maintained: false,
                confidence_level: 0.0,
                detected_faults: vec![FaultType::SensorFailure("Insufficient satellites".to_string())],
                protection_level: f64::INFINITY,
            };
        }

        // Compute test statistic (simplified)
        let mean: f64 = measurements.iter().sum::<f64>() / measurements.len() as f64;
        let variance: f64 = measurements.iter()
            .map(|x| (x - mean).powi(2))
            .sum::<f64>() / measurements.len() as f64;

        let test_stat = measurements.iter()
            .map(|x| ((x - mean) / variance.sqrt()).powi(2))
            .sum::<f64>();

        self.test_statistics.push(test_stat);

        let is_fault = test_stat > self.fault_detection_threshold;

        IntegrityResult {
            is_integrity_maintained: !is_fault,
            confidence_level: if is_fault { 0.0 } else { 0.95 },
            detected_faults: if is_fault {
                vec![FaultType::SensorFailure("Pseudorange outlier detected".to_string())]
            } else {
                vec![]
            },
            protection_level: self.compute_protection_level(measurements),
        }
    }
    fn compute_protection_level(&self, measurements: &[f64]) -> f64 {
        // Simplified protection level calculation
        // In practice, this would involve more complex RAIM algorithms
        let n = measurements.len() as f64;
        if n >= 5.0 {
            10.0 / n.sqrt() // Simplified formula
        } else {
            f64::INFINITY
        }
    }
}

/// FDE (Fault Detection and Exclusion) system
pub struct FDESystem {
    pub fault_hypothesis_tests: Vec<FaultHypothesis>,
    pub exclusion_threshold: f64,
    pub current_excluded_measurements: Vec<String>,
}

#[derive(Debug, Clone)]
pub struct FaultHypothesis {
    pub excluded_measurement: String,
    pub test_statistic: f64,
    pub probability: f64,
}

impl FDESystem {
    pub fn new() -> Self {
        FDESystem {
            fault_hypothesis_tests: Vec::new(),
            exclusion_threshold: 3.0,
            current_excluded_measurements: Vec::new(),
        }
    }

    /// Perform fault detection and exclusion
    pub fn perform_fde(&mut self, residuals: &Array1<f64>) -> Vec<String> {
        let mut excluded = Vec::new();

        for (i, &residual) in residuals.iter().enumerate() {
            if residual.abs() > self.exclusion_threshold {
                let measurement_id = format!("measurement_{}", i);
                excluded.push(measurement_id);
            }
        }

        self.current_excluded_measurements = excluded.clone();
        excluded
    }
}

/// Multi-constellation integrity monitoring
pub struct MultiConstellationMonitor {
    pub constellation_monitors: HashMap<String, RAIMMonitor>,
    pub cross_check_enabled: bool,
}

impl MultiConstellationMonitor {
    pub fn new() -> Self {
        let mut monitors = HashMap::new();
        monitors.insert("GPS".to_string(), RAIMMonitor::new());
        monitors.insert("GLONASS".to_string(), RAIMMonitor::new());
        monitors.insert("Galileo".to_string(), RAIMMonitor::new());
        monitors.insert("BeiDou".to_string(), RAIMMonitor::new());

        MultiConstellationMonitor {
            constellation_monitors: monitors,
            cross_check_enabled: true,
        }
    }

    /// Cross-check integrity across constellations
    pub fn cross_check_integrity(&self) -> IntegrityResult {
        let mut total_confidence = 0.0;
        let mut all_faults = Vec::new();
        let mut min_protection_level = f64::INFINITY;

        for (constellation, monitor) in &self.constellation_monitors {
            // Simplified cross-check logic
            if monitor.test_statistics.last().unwrap_or(&0.0) > monitor.fault_detection_threshold {
                all_faults.push(FaultType::SensorFailure(format!("{} integrity issue", constellation)));
            }
            total_confidence += 0.95; // Simplified
            min_protection_level = min_protection_level.min(monitor.protection_level);
        }

        let avg_confidence = total_confidence / self.constellation_monitors.len() as f64;

        IntegrityResult {
            is_integrity_maintained: all_faults.is_empty(),
            confidence_level: avg_confidence,
            detected_faults: all_faults,
            protection_level: min_protection_level,
        }
    }
}

/// Advanced integrity monitoring with machine learning
pub struct MLIntegrityMonitor {
    pub anomaly_detector: Option<AnomalyDetector>,
    pub prediction_model: Option<PredictionModel>,
    pub training_data: Vec<IntegrityTrainingSample>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IntegrityTrainingSample {
    pub sensor_data: SensorData,
    pub is_faulty: bool,
    pub fault_type: Option<FaultType>,
}

#[derive(Debug, Clone)]
pub struct AnomalyDetector {
    pub threshold: f64,
    pub features: Vec<String>,
}

#[derive(Debug, Clone)]
pub struct PredictionModel {
    pub weights: Array1<f64>,
    pub bias: f64,
}

impl MLIntegrityMonitor {
    pub fn new() -> Self {
        MLIntegrityMonitor {
            anomaly_detector: Some(AnomalyDetector {
                threshold: 0.8,
                features: vec!["residual".to_string(), "innovation".to_string()],
            }),
            prediction_model: None,
            training_data: Vec::new(),
        }
    }

    /// Detect anomalies using machine learning
    pub async fn detect_anomalies(&self, data: &SensorData) -> IntegrityResult {
        // Simplified anomaly detection
        // In practice, this would use trained ML models

        IntegrityResult {
            is_integrity_maintained: true, // Placeholder
            confidence_level: 0.9,
            detected_faults: Vec::new(),
            protection_level: 1.0,
        }
    }
}

impl IntegrityMonitor {
    pub fn new() -> Self {
        let mut monitors = HashMap::new();

        // Add various integrity checks
        monitors.insert("raim".to_string(), Box::new(RAIMMonitor::new()) as Box<dyn IntegrityCheck>);
        monitors.insert("fde".to_string(), Box::new(FDESystem::new()) as Box<dyn IntegrityCheck>);
        monitors.insert("multi_constellation".to_string(), Box::new(MultiConstellationMonitor::new()) as Box<dyn IntegrityCheck>);
        monitors.insert("ml_integrity".to_string(), Box::new(MLIntegrityMonitor::new()) as Box<dyn IntegrityCheck>);

        let mut alert_thresholds = HashMap::new();
        alert_thresholds.insert("raim".to_string(), 0.95);
        alert_thresholds.insert("fde".to_string(), 0.90);
        alert_thresholds.insert("multi_constellation".to_string(), 0.85);
        alert_thresholds.insert("ml_integrity".to_string(), 0.80);

        IntegrityMonitor {
            monitors,
            alert_thresholds,
            fault_history: Vec::new(),
            system_health: SystemHealth {
                overall_health: 1.0,
                subsystem_health: HashMap::new(),
                last_check_time: 0.0,
            },
        }
    }

    /// Perform comprehensive integrity monitoring
    pub async fn monitor_system(&mut self, data: &SensorData) -> SystemHealth {
        let mut total_health = 0.0;
        let mut subsystem_health = HashMap::new();

        for (name, monitor) in &self.monitors {
            match monitor.check_integrity(data).await {
                Ok(result) => {
                    let health = if result.is_integrity_maintained {
                        result.confidence_level
                    } else {
                        0.0
                    };

                    subsystem_health.insert(name.clone(), health);
                    total_health += health;

                    // Record faults
                    for fault in result.detected_faults {
                        self.fault_history.push(FaultEvent {
                            timestamp: std::time::SystemTime::now()
                                .duration_since(std::time::UNIX_EPOCH)
                                .unwrap()
                                .as_secs_f64(),
                            fault_type: fault,
                            severity: FaultSeverity::Moderate, // Default
                            affected_subsystems: vec![name.clone()],
                            mitigation_action: "Automatic exclusion".to_string(),
                        });
                    }
                }
                Err(e) => {
                    println!("Integrity check {} failed: {}", name, e);
                    subsystem_health.insert(name.clone(), 0.0);
                }
            }
        }

        let overall_health = total_health / self.monitors.len() as f64;

        self.system_health = SystemHealth {
            overall_health,
            subsystem_health,
            last_check_time: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_secs_f64(),
        };

        self.system_health.clone()
    }

    /// Get integrity alerts
    pub fn get_alerts(&self) -> Vec<String> {
        let mut alerts = Vec::new();

        for (subsystem, health) in &self.system_health.subsystem_health {
            if let Some(threshold) = self.alert_thresholds.get(subsystem) {
                if health < threshold {
                    alerts.push(format!("{} integrity compromised: {:.2}", subsystem, health));
                }
            }
        }

        alerts
    }
}

// Implement IntegrityCheck for each monitor type
#[async_trait::async_trait]
impl IntegrityCheck for RAIMMonitor {
    async fn check_integrity(&self, data: &SensorData) -> Result<IntegrityResult, String> {
        // Simplified implementation
        Ok(IntegrityResult {
            is_integrity_maintained: true,
            confidence_level: 0.95,
            detected_faults: Vec::new(),
            protection_level: 1.0,
        })
    }

    fn name(&self) -> &str {
        "RAIM"
    }
}

#[async_trait::async_trait]
impl IntegrityCheck for FDESystem {
    async fn check_integrity(&self, data: &SensorData) -> Result<IntegrityResult, String> {
        Ok(IntegrityResult {
            is_integrity_maintained: true,
            confidence_level: 0.90,
            detected_faults: Vec::new(),
            protection_level: 1.5,
        })
    }

    fn name(&self) -> &str {
        "FDE"
    }
}

#[async_trait::async_trait]
impl IntegrityCheck for MultiConstellationMonitor {
    async fn check_integrity(&self, data: &SensorData) -> Result<IntegrityResult, String> {
        Ok(self.cross_check_integrity())
    }

    fn name(&self) -> &str {
        "Multi-Constellation"
    }
}

#[async_trait::async_trait]
impl IntegrityCheck for MLIntegrityMonitor {
    async fn check_integrity(&self, data: &SensorData) -> Result<IntegrityResult, String> {
        // Use async block to call async method
        let result = self.detect_anomalies(data).await;
        Ok(result)
    }

    fn name(&self) -> &str {
        "ML Integrity"
    }
}

// Re-export key types
pub use crate::nif::architecture::SensorData;
use std::collections::HashMap;
use serde::{Deserialize, Serialize};
use reqwest::Client;
use std::time::Duration;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SparkJobConfig {
    pub job_name: String,
    pub job_type: SparkJobType,
    pub input_topic: String,
    pub output_path: String,
    pub batch_size: usize,
    pub parallelism: usize,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum SparkJobType {
    DataValidation,
    DataTransformation,
    DataAugmentation,
    FeatureExtraction,
    ModelTraining,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SparkJobResult {
    pub job_id: String,
    pub status: JobStatus,
    pub processed_records: u64,
    pub execution_time_ms: u64,
    pub output_path: String,
    pub metrics: HashMap<String, f64>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum JobStatus {
    Running,
    Completed,
    Failed,
    Cancelled,
}

pub struct SparkProcessor {
    spark_master_url: String,
    client: Client,
    active_jobs: HashMap<String, SparkJobResult>,
}

impl SparkProcessor {
    pub fn new(spark_master_url: String) -> Self {
        SparkProcessor {
            spark_master_url,
            client: Client::new(),
            active_jobs: HashMap::new(),
        }
    }

    pub async fn submit_job(&mut self, config: SparkJobConfig) -> Result<String, Box<dyn std::error::Error>> {
        let job_id = format!("spark_job_{}", chrono::Utc::now().timestamp());

        // Create Spark job specification
        let job_spec = serde_json::json!({
            "job_id": job_id,
            "job_name": config.job_name,
            "job_type": format!("{:?}", config.job_type),
            "input_topic": config.input_topic,
            "output_path": config.output_path,
            "batch_size": config.batch_size,
            "parallelism": config.parallelism,
            "submit_time": chrono::Utc::now().to_rfc3339()
        });

        // Submit job to Spark master
        let submit_url = format!("{}/jobs/submit", self.spark_master_url);
        let response = self.client
            .post(&submit_url)
            .json(&job_spec)
            .send()
            .await?;

        if response.status().is_success() {
            let result = SparkJobResult {
                job_id: job_id.clone(),
                status: JobStatus::Running,
                processed_records: 0,
                execution_time_ms: 0,
                output_path: config.output_path.clone(),
                metrics: HashMap::new(),
            };

            self.active_jobs.insert(job_id.clone(), result);
            println!("Submitted Spark job: {}", job_id);
            Ok(job_id)
        } else {
            Err(format!("Failed to submit Spark job: {}", response.status()).into())
        }
    }

    pub async fn get_job_status(&self, job_id: &str) -> Result<Option<SparkJobResult>, Box<dyn std::error::Error>> {
        let status_url = format!("{}/jobs/{}/status", self.spark_master_url, job_id);
        let response = self.client.get(&status_url).send().await?;

        if response.status().is_success() {
            let status_data: serde_json::Value = response.json().await?;
            let status = match status_data["status"].as_str() {
                Some("RUNNING") => JobStatus::Running,
                Some("COMPLETED") => JobStatus::Completed,
                Some("FAILED") => JobStatus::Failed,
                Some("CANCELLED") => JobStatus::Cancelled,
                _ => JobStatus::Running,
            };

            let result = SparkJobResult {
                job_id: job_id.to_string(),
                status,
                processed_records: status_data["processed_records"].as_u64().unwrap_or(0),
                execution_time_ms: status_data["execution_time_ms"].as_u64().unwrap_or(0),
                output_path: status_data["output_path"].as_str().unwrap_or("").to_string(),
                metrics: HashMap::new(), // Would parse actual metrics
            };

            Ok(Some(result))
        } else {
            Ok(None)
        }
    }

    pub async fn cancel_job(&mut self, job_id: &str) -> Result<(), Box<dyn std::error::Error>> {
        let cancel_url = format!("{}/jobs/{}/cancel", self.spark_master_url, job_id);
        let response = self.client.post(&cancel_url).send().await?;

        if response.status().is_success() {
            if let Some(job) = self.active_jobs.get_mut(job_id) {
                job.status = JobStatus::Cancelled;
            }
            println!("Cancelled Spark job: {}", job_id);
            Ok(())
        } else {
            Err(format!("Failed to cancel Spark job: {}", response.status()).into())
        }
    }

    pub async fn validate_dataset(&mut self, dataset_name: &str, input_topic: &str) -> Result<String, Box<dyn std::error::Error>> {
        let config = SparkJobConfig {
            job_name: format!("validate_{}", dataset_name),
            job_type: SparkJobType::DataValidation,
            input_topic: input_topic.to_string(),
            output_path: format!("/validated/{}", dataset_name),
            batch_size: 1000,
            parallelism: 4,
        };

        self.submit_job(config).await
    }

    pub async fn transform_dataset(&mut self, dataset_name: &str, input_topic: &str, transformation: &str) -> Result<String, Box<dyn std::error::Error>> {
        let config = SparkJobConfig {
            job_name: format!("transform_{}_{}", dataset_name, transformation),
            job_type: SparkJobType::DataTransformation,
            input_topic: input_topic.to_string(),
            output_path: format!("/transformed/{}/{}", dataset_name, transformation),
            batch_size: 1000,
            parallelism: 8,
        };

        self.submit_job(config).await
    }

    pub async fn augment_dataset(&mut self, dataset_name: &str, input_topic: &str) -> Result<String, Box<dyn std::error::Error>> {
        let config = SparkJobConfig {
            job_name: format!("augment_{}", dataset_name),
            job_type: SparkJobType::DataAugmentation,
            input_topic: input_topic.to_string(),
            output_path: format!("/augmented/{}", dataset_name),
            batch_size: 500,
            parallelism: 6,
        };

        self.submit_job(config).await
    }

    pub async fn extract_features(&mut self, dataset_name: &str, input_path: &str) -> Result<String, Box<dyn std::error::Error>> {
        let config = SparkJobConfig {
            job_name: format!("features_{}", dataset_name),
            job_type: SparkJobType::FeatureExtraction,
            input_topic: input_path.to_string(), // Using input_topic for path in this context
            output_path: format!("/features/{}", dataset_name),
            batch_size: 2000,
            parallelism: 12,
        };

        self.submit_job(config).await
    }

    pub async fn get_cluster_metrics(&self) -> Result<SparkClusterMetrics, Box<dyn std::error::Error>> {
        let metrics_url = format!("{}/metrics", self.spark_master_url);
        let response = self.client.get(&metrics_url).send().await?;

        if response.status().is_success() {
            let metrics_data: serde_json::Value = response.json().await?;
            let metrics = SparkClusterMetrics {
                active_jobs: metrics_data["active_jobs"].as_u64().unwrap_or(0),
                completed_jobs: metrics_data["completed_jobs"].as_u64().unwrap_or(0),
                failed_jobs: metrics_data["failed_jobs"].as_u64().unwrap_or(0),
                total_cores: metrics_data["total_cores"].as_u64().unwrap_or(0),
                used_cores: metrics_data["used_cores"].as_u64().unwrap_or(0),
                total_memory_gb: metrics_data["total_memory_gb"].as_f64().unwrap_or(0.0),
                used_memory_gb: metrics_data["used_memory_gb"].as_f64().unwrap_or(0.0),
            };
            Ok(metrics)
        } else {
            Err(format!("Failed to get cluster metrics: {}", response.status()).into())
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SparkClusterMetrics {
    pub active_jobs: u64,
    pub completed_jobs: u64,
    pub failed_jobs: u64,
    pub total_cores: u64,
    pub used_cores: u64,
    pub total_memory_gb: f64,
    pub used_memory_gb: f64,
}

pub async fn initialize_spark_processor(spark_master_url: String) -> Result<SparkProcessor, Box<dyn std::error::Error>> {
    println!("Initializing Spark processor with master URL: {}", spark_master_url);
    let processor = SparkProcessor::new(spark_master_url);

    // Test connection
    match processor.get_cluster_metrics().await {
        Ok(metrics) => {
            println!("Spark cluster connected - Cores: {}, Memory: {:.1}GB",
                    metrics.total_cores, metrics.total_memory_gb);
            Ok(processor)
        }
        Err(e) => {
            println!("Warning: Could not connect to Spark cluster: {}", e);
            // Return processor anyway for offline mode
            Ok(processor)
        }
    }
}

pub async fn run_data_pipeline_demo(processor: &mut SparkProcessor, kafka_pipeline: &super::kafka_pipeline::KafkaDataPipeline) -> Result<(), Box<dyn std::error::Error>> {
    println!("Running data pipeline demo...");

    // Validate datasets
    let waymo_job = processor.validate_dataset("waymo", "waymo-dataset").await?;
    let kitti_job = processor.validate_dataset("kitti", "kitti-dataset").await?;
    let rtx_job = processor.validate_dataset("rtx", "rtx-dataset").await?;

    println!("Submitted validation jobs: {}, {}, {}", waymo_job, kitti_job, rtx_job);

    // Wait a bit for jobs to start
    tokio::time::sleep(Duration::from_secs(2)).await;

    // Check job statuses
    if let Ok(Some(status)) = processor.get_job_status(&waymo_job).await {
        println!("Waymo validation status: {:?}", status.status);
    }

    // Transform validated data
    let transform_job = processor.transform_dataset("waymo", "waymo-dataset", "normalize").await?;
    println!("Submitted transformation job: {}", transform_job);

    // Augment data for training
    let augment_job = processor.augment_dataset("waymo", "waymo-dataset").await?;
    println!("Submitted augmentation job: {}", augment_job);

    // Extract features
    let features_job = processor.extract_features("waymo", "/transformed/waymo/normalize").await?;
    println!("Submitted feature extraction job: {}", features_job);

    Ok(())
}
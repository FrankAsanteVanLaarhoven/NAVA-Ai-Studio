use rdkafka::config::ClientConfig;
use rdkafka::producer::{FutureProducer, FutureRecord};
use rdkafka::consumer::{Consumer, StreamConsumer};
use rdkafka::message::Message;
use std::time::Duration;
use tokio::time;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct DatasetRecord {
    pub dataset: String,
    pub record_id: String,
    pub data: Vec<u8>,
    pub timestamp: u64,
    pub metadata: HashMap<String, String>,
}

pub struct KafkaDataPipeline {
    producer: FutureProducer,
    consumer: StreamConsumer,
    topics: Vec<String>,
}

impl KafkaDataPipeline {
    pub fn new(brokers: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let producer: FutureProducer = ClientConfig::new()
            .set("bootstrap.servers", brokers)
            .set("message.timeout.ms", "5000")
            .create()?;

        let consumer: StreamConsumer = ClientConfig::new()
            .set("group.id", "navlambda-dataset-consumer")
            .set("bootstrap.servers", brokers)
            .set("enable.partition.eof", "false")
            .set("session.timeout.ms", "6000")
            .set("enable.auto.commit", "true")
            .create()?;

        let topics = vec![
            "waymo-dataset".to_string(),
            "kitti-dataset".to_string(),
            "rtx-dataset".to_string(),
        ];

        Ok(KafkaDataPipeline {
            producer,
            consumer,
            topics,
        })
    }

    pub async fn initialize_topics(&self) -> Result<(), Box<dyn std::error::Error>> {
        // Create topics if they don't exist
        for topic in &self.topics {
            println!("Ensuring topic exists: {}", topic);
            // In a real implementation, you would use the Kafka Admin API
            // For now, we'll assume topics are pre-created
        }
        Ok(())
    }

    pub async fn produce_record(&self, topic: &str, record: DatasetRecord) -> Result<(), Box<dyn std::error::Error>> {
        let payload = serde_json::to_string(&record)?;
        let record = FutureRecord::to(topic)
            .payload(&payload)
            .key(&record.record_id);

        let delivery_status = self.producer.send(record, Duration::from_secs(0)).await;
        match delivery_status {
            Ok(_) => Ok(()),
            Err((e, _)) => Err(Box::new(e)),
        }
    }

    pub async fn consume_records(&self, topic: &str) -> Result<Vec<DatasetRecord>, Box<dyn std::error::Error>> {
        self.consumer.subscribe(&[topic])?;

        let mut records = Vec::new();
        let mut timeout = time::timeout(Duration::from_secs(5), async {
            let mut count = 0;
            while count < 100 { // Limit to 100 records per batch
                match self.consumer.recv().await {
                    Ok(message) => {
                        if let Some(payload) = message.payload() {
                            if let Ok(record) = serde_json::from_slice::<DatasetRecord>(payload) {
                                records.push(record);
                                count += 1;
                            }
                        }
                    }
                    Err(e) => {
                        println!("Error receiving message: {}", e);
                        break;
                    }
                }
            }
        });

        let _ = timeout.await;
        Ok(records)
    }

    pub async fn stream_to_spark(&self, spark_endpoint: &str) -> Result<(), Box<dyn std::error::Error>> {
        for topic in &self.topics {
            let records = self.consume_records(topic).await?;
            if !records.is_empty() {
                // Send batch to Spark for processing
                println!("Streaming {} records from {} to Spark", records.len(), topic);
                // In a real implementation, this would send data to Spark via REST API or Kafka-Spark integration
            }
        }
        Ok(())
    }

    pub async fn get_topic_stats(&self) -> Result<HashMap<String, TopicStats>, Box<dyn std::error::Error>> {
        let mut stats = HashMap::new();

        for topic in &self.topics {
            // In a real implementation, you would query Kafka for topic statistics
            let topic_stats = TopicStats {
                message_count: 0, // Would be retrieved from Kafka
                partitions: 3,
                replication_factor: 3,
                throughput_mbps: 0.0,
            };
            stats.insert(topic.clone(), topic_stats);
        }

        Ok(stats)
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct TopicStats {
    pub message_count: u64,
    pub partitions: i32,
    pub replication_factor: i32,
    pub throughput_mbps: f64,
}

pub async fn initialize_kafka_pipeline(brokers: &str) -> Result<KafkaDataPipeline, Box<dyn std::error::Error>> {
    println!("Initializing Kafka data pipeline with brokers: {}", brokers);
    let pipeline = KafkaDataPipeline::new(brokers)?;
    pipeline.initialize_topics().await?;
    println!("Kafka pipeline initialized successfully");
    Ok(pipeline)
}

pub async fn test_kafka_throughput(pipeline: &KafkaDataPipeline) -> Result<f64, Box<dyn std::error::Error>> {
    let test_topic = "waymo-dataset";
    let mut throughput = 0.0;

    // Generate test records
    let start_time = std::time::Instant::now();
    let mut record_count = 0;

    for i in 0..1000 {
        let record = DatasetRecord {
            dataset: "waymo".to_string(),
            record_id: format!("test_record_{}", i),
            data: vec![0; 1024], // 1KB test data
            timestamp: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_secs(),
            metadata: HashMap::new(),
        };

        if let Ok(_) = pipeline.produce_record(test_topic, record).await {
            record_count += 1;
        }
    }

    let elapsed = start_time.elapsed().as_secs_f64();
    throughput = record_count as f64 / elapsed;

    println!("Kafka throughput test: {:.2} records/sec", throughput);
    Ok(throughput)
}
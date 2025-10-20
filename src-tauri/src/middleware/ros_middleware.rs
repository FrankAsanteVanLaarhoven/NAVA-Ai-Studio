
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::broadcast;
use parking_lot::RwLock;
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ROSMessage {
    pub topic: String,
    pub data: serde_json::Value,
    pub timestamp: u64,
}

pub type MessageCallback = Box<dyn Fn(ROSMessage) + Send + Sync>;

pub struct ROSTopic {
    subscribers: Vec<MessageCallback>,
    last_message: Option<ROSMessage>,
}

impl ROSTopic {
    pub fn new() -> Self {
        ROSTopic {
            subscribers: Vec::new(),
            last_message: None,
        }
    }

    pub fn publish(&mut self, message: ROSMessage) {
        self.last_message = Some(message.clone());
        for callback in &self.subscribers {
            callback(message.clone());
        }
    }

    pub fn subscribe(&mut self, callback: MessageCallback) {
        self.subscribers.push(callback);
    }

    pub fn get_last_message(&self) -> Option<&ROSMessage> {
        self.last_message.as_ref()
    }
}

pub struct ROSMiddleware {
    topics: HashMap<String, Arc<RwLock<ROSTopic>>>,
    nodes: HashMap<String, String>, // node_name -> node_type
}

impl ROSMiddleware {
    pub fn new() -> Self {
        ROSMiddleware {
            topics: HashMap::new(),
            nodes: HashMap::new(),
        }
    }

    pub fn create_topic(&mut self, topic_name: String) {
        self.topics.insert(topic_name, Arc::new(RwLock::new(ROSTopic::new())));
    }

    pub fn publish_message(&mut self, topic: &str, message: ROSMessage) -> Result<(), String> {
        if let Some(topic_ref) = self.topics.get(topic) {
            let mut topic_lock = topic_ref.write();
            topic_lock.publish(message);
            Ok(())
        } else {
            Err(format!("Topic '{}' does not exist", topic))
        }
    }

    pub fn subscribe_to_topic<F>(&mut self, topic: &str, callback: F) -> Result<(), String>
    where
        F: Fn(ROSMessage) + Send + Sync + 'static,
    {
        if let Some(topic_ref) = self.topics.get(topic) {
            let mut topic_lock = topic_ref.write();
            topic_lock.subscribe(Box::new(callback));
            Ok(())
        } else {
            Err(format!("Topic '{}' does not exist", topic))
        }
    }

    pub fn register_node(&mut self, node_name: String, node_type: String) {
        self.nodes.insert(node_name, node_type);
        println!("Registered ROS2 node: {} ({})", node_name, node_type);
    }

    pub fn get_topics(&self) -> Vec<String> {
        self.topics.keys().cloned().collect()
    }

    pub fn get_nodes(&self) -> Vec<(String, String)> {
        self.nodes.iter().map(|(k, v)| (k.clone(), v.clone())).collect()
    }
}

static mut MIDDLEWARE: Option<ROSMiddleware> = None;

pub fn initialize() -> Result<(), Box<dyn std::error::Error>> {
    unsafe {
        MIDDLEWARE = Some(ROSMiddleware::new());
    }
    println!("Initializing ROS2 Middleware Stack...");

    // Create some default topics
    if let Some(ref mut mw) = unsafe { MIDDLEWARE.as_mut() } {
        mw.create_topic("/cmd_vel".to_string());
        mw.create_topic("/odom".to_string());
        mw.create_topic("/scan".to_string());
        mw.create_topic("/camera/image_raw".to_string());
    }

    Ok(())
}

pub fn publish_message(topic: &str, message: &str) -> Result<(), String> {
    let ros_message = ROSMessage {
        topic: topic.to_string(),
        data: serde_json::from_str(message).map_err(|e| format!("Invalid JSON: {}", e))?,
        timestamp: std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_secs(),
    };

    if let Some(ref mut mw) = unsafe { MIDDLEWARE.as_mut() } {
        mw.publish_message(topic, ros_message)
    } else {
        Err("Middleware not initialized".to_string())
    }
}

pub fn subscribe_to_topic(topic: &str) -> Result<(), String> {
    if let Some(ref mut mw) = unsafe { MIDDLEWARE.as_mut() } {
        mw.subscribe_to_topic(topic, |msg| {
            println!("Received message on topic '{}': {:?}", msg.topic, msg.data);
        })
    } else {
        Err("Middleware not initialized".to_string())
    }
}

pub fn get_middleware_info() -> Result<(Vec<String>, Vec<(String, String)>), String> {
    if let Some(ref mw) = unsafe { MIDDLEWARE.as_ref() } {
        Ok((mw.get_topics(), mw.get_nodes()))
    } else {
        Err("Middleware not initialized".to_string())
    }
}

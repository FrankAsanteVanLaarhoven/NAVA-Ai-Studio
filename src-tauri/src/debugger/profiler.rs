use std::time::{Duration, Instant};
use std::collections::HashMap;

/// Performance Profiler for VNC Operations
pub struct VncProfiler {
    profiles: HashMap<String, ProfileData>,
    current_profile: Option<(String, Instant)>,
}

#[derive(Debug, Clone)]
pub struct ProfileData {
    pub total_time: Duration,
    pub call_count: usize,
    pub avg_time: Duration,
    pub max_time: Duration,
    pub min_time: Duration,
}

impl VncProfiler {
    pub fn new() -> Self {
        Self {
            profiles: HashMap::new(),
            current_profile: None,
        }
    }
    
    pub fn start_profile(&mut self, name: &str) {
        self.current_profile = Some((name.to_string(), Instant::now()));
    }
    
    pub fn end_profile(&mut self) {
        if let Some((name, start_time)) = self.current_profile.take() {
            let duration = start_time.elapsed();
            
            self.profiles.entry(name.clone())
                .and_modify(|profile| {
                    profile.total_time += duration;
                    profile.call_count += 1;
                    profile.avg_time = profile.total_time / profile.call_count as u32;
                    profile.max_time = profile.max_time.max(duration);
                    profile.min_time = profile.min_time.min(duration);
                })
                .or_insert(ProfileData {
                    total_time: duration,
                    call_count: 1,
                    avg_time: duration,
                    max_time: duration,
                    min_time: duration,
                });
        }
    }
    
    pub fn get_profile(&self, name: &str) -> Option<&ProfileData> {
        self.profiles.get(name)
    }
    
    pub fn get_all_profiles(&self) -> &HashMap<String, ProfileData> {
        &self.profiles
    }
    
    pub fn clear(&mut self) {
        self.profiles.clear();
        self.current_profile = None;
    }
}

impl Default for VncProfiler {
    fn default() -> Self {
        Self::new()
    }
}


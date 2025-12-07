// NAVΛ prefix (Rust)
pub struct RtScope<'a> { /* hooks for metrics, logging */ }
impl<'a> RtScope<'a> { pub fn record(&self, k:&str, v:f64) { /* ... */ } }

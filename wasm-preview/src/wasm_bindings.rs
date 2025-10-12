use wasm_bindgen::prelude::*;

/// JavaScript Bindings for WASM Preview

#[wasm_bindgen]
extern "C" {
    #[wasm_bindgen(js_namespace = console)]
    pub fn log(s: &str);
    
    #[wasm_bindgen(js_namespace = console)]
    pub fn error(s: &str);
}

#[wasm_bindgen]
pub fn initialize_bindings() {
    log("WASM bindings initialized");
}

#[wasm_bindgen]
pub struct JsNavigationPath {
    inner: super::navigation_runtime::NavigationPath,
}

#[wasm_bindgen]
impl JsNavigationPath {
    pub fn get_energy(&self) -> f64 {
        self.inner.energy
    }
    
    pub fn waypoint_count(&self) -> usize {
        self.inner.waypoints.len()
    }
}


// Prevents additional console window on Windows in release
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

mod lsp;
mod compiler;
mod debugger;
mod preview;
mod plugins;

use std::sync::Arc;
use parking_lot::RwLock;
use tracing_subscriber;

// Application state
pub struct AppState {
    lsp_server: Arc<RwLock<lsp::NavLambdaLanguageServer>>,
    compiler: Arc<compiler::MultiTargetCompiler>,
    debugger: Arc<debugger::NavigationDebugger>,
    preview_engine: Arc<preview::LivePreviewEngine>,
    plugin_manager: Arc<plugins::PluginManager>,
}

// Tauri commands
#[tauri::command]
async fn initialize_lsp(state: tauri::State<'_, AppState>) -> Result<String, String> {
    let server = state.lsp_server.read();
    Ok(format!("NAVΛ Language Server initialized with VNC support"))
}

#[tauri::command]
async fn parse_navlambda_code(
    code: String,
    state: tauri::State<'_, AppState>,
) -> Result<String, String> {
    let server = state.lsp_server.read();
    match server.parser.parse(&code) {
        Ok(ast) => Ok(format!("Parsed successfully: {} statements", ast.statements.len())),
        Err(e) => Err(format!("Parse error: {}", e)),
    }
}

#[tauri::command]
async fn compile_to_target(
    code: String,
    target: String,
    state: tauri::State<'_, AppState>,
) -> Result<String, String> {
    let server = state.lsp_server.read();
    let ast = server.parser.parse(&code).map_err(|e| format!("Parse error: {}", e))?;
    
    let compilation_target = match target.as_str() {
        "cpp" => compiler::CompilationTarget::Cpp17,
        "python" => compiler::CompilationTarget::Python312,
        "wasm" => compiler::CompilationTarget::WebAssembly,
        "glsl" => compiler::CompilationTarget::Glsl450,
        _ => return Err(format!("Unknown target: {}", target)),
    };
    
    match state.compiler.compile_target(&ast, compilation_target).await {
        Ok(output) => Ok(output.code),
        Err(e) => Err(format!("Compilation error: {}", e)),
    }
}

#[tauri::command]
async fn get_code_completions(
    code: String,
    position: usize,
    state: tauri::State<'_, AppState>,
) -> Result<Vec<String>, String> {
    let server = state.lsp_server.read();
    Ok(server.get_completions(&code, position))
}

#[tauri::command]
async fn get_hover_info(
    code: String,
    position: usize,
    state: tauri::State<'_, AppState>,
) -> Result<String, String> {
    let server = state.lsp_server.read();
    Ok(server.get_hover_info(&code, position))
}

#[tauri::command]
async fn visualize_navigation_path(
    code: String,
    state: tauri::State<'_, AppState>,
) -> Result<String, String> {
    let server = state.lsp_server.read();
    let ast = server.parser.parse(&code).map_err(|e| format!("Parse error: {}", e))?;
    
    let visualization = state.debugger.trace_navigation_path(&ast);
    Ok(serde_json::to_string(&visualization).unwrap())
}

#[tauri::command]
async fn run_live_preview(
    code: String,
    state: tauri::State<'_, AppState>,
) -> Result<String, String> {
    let server = state.lsp_server.read();
    let ast = server.parser.parse(&code).map_err(|e| format!("Parse error: {}", e))?;
    
    match state.preview_engine.execute(&ast).await {
        Ok(result) => Ok(result),
        Err(e) => Err(format!("Execution error: {}", e)),
    }
}

fn main() {
    // Initialize tracing
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::INFO)
        .init();

    tracing::info!("🚀 Starting NAVΛ Studio IDE...");

    // Initialize application state
    let app_state = AppState {
        lsp_server: Arc::new(RwLock::new(lsp::NavLambdaLanguageServer::new())),
        compiler: Arc::new(compiler::MultiTargetCompiler::new()),
        debugger: Arc::new(debugger::NavigationDebugger::new()),
        preview_engine: Arc::new(preview::LivePreviewEngine::new()),
        plugin_manager: Arc::new(plugins::PluginManager::new()),
    };

    tauri::Builder::default()
        .manage(app_state)
        .invoke_handler(tauri::generate_handler![
            initialize_lsp,
            parse_navlambda_code,
            compile_to_target,
            get_code_completions,
            get_hover_info,
            visualize_navigation_path,
            run_live_preview,
        ])
        .run(tauri::generate_context!())
        .expect("error while running NAVΛ Studio application");
}


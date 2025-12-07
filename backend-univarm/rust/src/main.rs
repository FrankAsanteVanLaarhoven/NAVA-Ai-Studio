use actix_cors::Cors;
use actix_files::NamedFile;
use actix_web::{web, App, Error, HttpRequest, HttpResponse, HttpServer, Responder};
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use parking_lot::RwLock;
use tokio::sync::broadcast;
use uuid::Uuid;

mod solver;

#[derive(Clone)]
struct AppState {
    tx: broadcast::Sender<SseEvent>,
    presets: Arc<RwLock<serde_json::Value>>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct SolveRequest {
    start: [f64; 2],
    goal: [f64; 2],
    samples: Option<u32>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct PathPoint {
    x: f64,
    y: f64,
    theta: Option<f64>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct PathRequest {
    path: Vec<PathPoint>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct SolveResponse {
    path: Vec<PathPoint>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct TrustSummary {
    pass: bool,
    metrics: serde_json::Value,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct SseEvent {
    kind: String,
    data: serde_json::Value,
}

async fn solve(
    state: web::Data<AppState>,
    payload: web::Json<SolveRequest>,
) -> actix_web::Result<impl Responder> {
    let path = solver::solve_path(payload.into_inner()).await.map_err(|e| {
        actix_web::error::ErrorInternalServerError(format!("solver error: {e:?}"))
    })?;
    Ok(web::Json(SolveResponse { path }))
}

async fn drive_path(
    state: web::Data<AppState>,
    payload: web::Json<PathRequest>,
) -> actix_web::Result<impl Responder> {
    // broadcast a job-start and spawn a simple wheel simulation along the path
    let job_id = Uuid::new_v4().to_string();
    let tx = state.tx.clone();
    let path = payload.path.clone();
    tokio::spawn(async move {
        let mut t = 0.0f64;
        for (i, p) in path.iter().enumerate() {
            // Fake wheel velocities derived from curvature-ish heuristic
            let v = 0.4 + 0.2 * ((i as f64) / (path.len().max(1) as f64));
            let w = p.theta.unwrap_or(0.0).sin() * 0.5;
            let satisfaction = (1.0 - (w.abs() / 1.0)).clamp(0.0, 1.0);
            let _ = tx.send(SseEvent {
                kind: "wheels".into(),
                data: serde_json::json!({
                    "v": v, "w": w,
                    "left_w": v - 0.5*w,
                    "right_w": v + 0.5*w,
                    "constraint_satisfaction": satisfaction,
                    "t": t,
                    "job": job_id,
                }),
            });
            t += 0.05;
            tokio::time::sleep(std::time::Duration::from_millis(50)).await;
        }
        let _ = tx.send(SseEvent {
            kind: "complete".into(),
            data: serde_json::json!({ "job": job_id }),
        });
    });
    Ok(web::Json(serde_json::json!({ "job": job_id })))
}

async fn sse(
    req: HttpRequest,
    state: web::Data<AppState>,
) -> Result<HttpResponse, Error> {
    // Token is accepted but not enforced in this starter
    let mut rx = state.tx.subscribe();
    let (tx, body) = actix_web::body::BodyStream::new();
    tokio::spawn(async move {
        use futures_util::stream::StreamExt;
        let mut sender = tx;
        loop {
            match rx.recv().await {
                Ok(evt) => {
                    let line = format!("event: {}\ndata: {}\n\n", evt.kind, serde_json::to_string(&evt.data).unwrap());
                    if sender.send(Ok(web::Bytes::from(line))).await.is_err() {
                        break;
                    }
                }
                Err(_) => break,
            }
        }
    });
    Ok(HttpResponse::Ok()
        .append_header(("Content-Type", "text/event-stream"))
        .append_header(("Cache-Control", "no-cache"))
        .append_header(("Connection", "keep-alive"))
        .body(body))
}

async fn presets_list(state: web::Data<AppState>) -> actix_web::Result<impl Responder> {
    Ok(web::Json(state.presets.read().clone()))
}

async fn actions_catalog() -> actix_web::Result<impl Responder> {
    Ok(web::Json(serde_json::json!({
        "actions": [
            {"id":"solve_path","label":"Solve path (⋋)"},
            {"id":"emit_ros2_path","label":"Emit ROS2 nav_msgs/Path"},
            {"id":"push_xr","label":"Push to XR Bench"}
        ]
    })))
}

async fn trust_summary() -> actix_web::Result<impl Responder> {
    Ok(web::Json(TrustSummary {
        pass: true,
        metrics: serde_json::json!({
            "dmr": 0.0, "aj": 0.0, "ttp_ms": 0
        }),
    }))
}

async fn static_engine() -> actix_web::Result<NamedFile> {
    // Placeholder so 404s don't spam if engine isn't provided yet
    NamedFile::open_async("public/engine/README.md").await.map_err(|e| actix_web::error::ErrorNotFound(e))
}

#[actix_web::main]
async fn main() -> std::io::Result<()> {
    let host = std::env::var("HOST").unwrap_or_else(|_| "0.0.0.0".into());
    let port: u16 = std::env::var("PORT").ok().and_then(|p| p.parse().ok()).unwrap_or(8080);

    let (tx, _rx) = broadcast::channel::<SseEvent>(128);
    let presets_json = include_str!("presets.json");
    let presets_val: serde_json::Value = serde_json::from_str(presets_json).unwrap_or(serde_json::json!({"presets":[]}));

    let state = AppState {
        tx,
        presets: Arc::new(RwLock::new(presets_val))
    };

    println!("Starting Univarm backend on {host}:{port}");
    HttpServer::new(move || {
        let cors = Cors::default()
            .allow_any_method()
            .allow_any_header()
            .supports_credentials()
            .allowed_origin("http://localhost:3000")
            .allowed_origin("http://localhost:5175");

        App::new()
            .app_data(web::Data::new(state.clone()))
            .wrap(cors)
            .route("/api/solve", web::post().to(solve))
            .route("/api/drive/path", web::post().to(drive_path))
            .route("/api/drive/sse", web::get().to(sse))
            .route("/api/ui/presets", web::get().to(presets_list))
            .route("/api/actions/catalog", web::get().to(actions_catalog))
            .route("/api/trust/summary", web::get().to(trust_summary))
            .route("/engine/navlambda_engine.wasm", web::get().to(static_engine))
    })
    .bind((host.as_str(), port))?
    .run()
    .await
}

use anyhow::{anyhow, Result};
use serde::{Deserialize, Serialize};

#[cfg(feature = "solver-wasm")]
mod wasm;
#[cfg(feature = "solver-grpc")]
mod grpc;

use super::super::{SolveRequest, PathPoint}; // Not available; we re-declare locally

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SolveReq {
    pub start: [f64; 2],
    pub goal: [f64; 2],
    pub samples: Option<u32>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SolveResp {
    pub path: Vec<PathPoint>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PathPoint {
    pub x: f64,
    pub y: f64,
    pub theta: Option<f64>,
}

pub async fn solve_path(req: crate::SolveRequest) -> Result<Vec<crate::PathPoint>> {
    // Dispatch depending on env
    let mode = std::env::var("NAVL_SOLVER").unwrap_or_else(|_| "wasm".into());
    match mode.as_str() {
        "grpc" => {
            #[cfg(feature = "solver-grpc")]
            {
                let r = grpc::solve(req.start, req.goal, req.samples.unwrap_or(200)).await?;
                return Ok(r.into_iter().map(|(x,y)| crate::PathPoint { x, y, theta: None }).collect());
            }
            #[cfg(not(feature = "solver-grpc"))]
            {
                return Err(anyhow!("compiled without solver-grpc feature"));
            }
        }
        _ => {
            #[cfg(feature = "solver-wasm")]
            {
                let r = wasm::solve(req.start, req.goal, req.samples.unwrap_or(200)).await?;
                return Ok(r.into_iter().map(|(x,y)| crate::PathPoint { x, y, theta: None }).collect());
            }
            #[cfg(not(feature = "solver-wasm"))]
            {
                return Err(anyhow!("compiled without solver-wasm feature"));
            }
        }
    }
}

#[cfg(feature = "solver-wasm")]
mod wasm {
    use super::*;
    use anyhow::Result;
    use wasmtime::{Engine, Linker, Module, Store, Caller, TypedFunc};

    pub async fn solve(start: [f64;2], goal: [f64;2], samples: u32) -> Result<Vec<(f64,f64)>> {
        // Load engine.wasm from ./public/engine/ if present
        let path = std::env::var("NAVL_WASM_PATH").unwrap_or_else(|_| "public/engine/navlambda_engine.wasm".into());
        let engine = Engine::default();
        let module = Module::from_file(&engine, path)
            .map_err(|e| anyhow::anyhow!("failed to load WASM: {e}"))?;
        let mut store = Store::new(&engine, ());
        let mut linker = Linker::new(&engine);
        let instance = linker.instantiate(&mut store, &module)?;

        // Expect exported `solve_lerp` as a placeholder: (f64,f64,f64,f64,i32) -> i32 (points stored in linear buffer and read via memory)
        // For a real engine, replace with your exported signature.
        // Here we fallback to a simple lerp in Rust if symbol not found.
        if let Ok(func) = instance.get_typed_func::<(f64,f64,f64,f64,i32), i32, _>(&mut store, "solve_lerp") {
            let n = samples as i32;
            let _ = func.call(&mut store, (start[0], start[1], goal[0], goal[1], n))?;
            // For starter we still return a lerp path from Rust to avoid memory mgmt complexity:
        }
        // Fallback: local lerp
        let n = samples.max(2) as usize;
        let mut out = Vec::with_capacity(n);
        for i in 0..n {
            let t = (i as f64) / ((n-1) as f64);
            out.push((start[0]*(1.0-t)+goal[0]*t, start[1]*(1.0-t)+goal[1]*t));
        }
        Ok(out)
    }
}

#[cfg(feature = "solver-grpc")]
mod grpc {
    use super::*;
    use tonic::transport::Channel;
    use tonic::Request;
    pub mod drive {
        tonic::include_proto!("drive");
    }
    use drive::{drive_client::DriveClient, SolvePathRequest};

    pub async fn solve(start: [f64;2], goal: [f64;2], samples: u32) -> anyhow::Result<Vec<(f64,f64)>> {
        let uri = std::env::var("PLANNER_ADDR").unwrap_or_else(|_| "http://127.0.0.1:50051".into());
        let mut client = DriveClient::connect(uri).await?;
        let req = SolvePathRequest {
            start: Some(drive::Vec2{ x: start[0], y: start[1] }),
            goal: Some(drive::Vec2{ x: goal[0], y: goal[1] }),
            samples: samples as i32,
        };
        let resp = client.solve_path(Request::new(req)).await?.into_inner();
        let pts = resp.points.into_iter().map(|p| (p.x, p.y)).collect();
        Ok(pts)
    }
}

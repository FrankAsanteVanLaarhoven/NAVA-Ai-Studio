# Univarm Backend (Rust)

- **Two solver modes** (pick with env `NAVL_SOLVER=`):
  - `wasm` (default): loads a NAVΛ engine at `public/engine/navlambda_engine.wasm` via Wasmtime.
  - `grpc`: calls a planner daemon implementing `Drive.SolvePath` (see `Drive.proto`).

- **Endpoints**
  - `POST /api/solve` → `{start:[x,y], goal:[x,y], samples}` → `{path:[{x,y,theta?}]}`
  - `POST /api/drive/path` → `{path:[...]}` → `{job}` (starts simulated wheel loop)
  - `GET  /api/drive/sse` → `event:wheels` JSON, includes `constraint_satisfaction`
  - `GET  /api/ui/presets`
  - `GET  /api/actions/catalog`
  - `GET  /api/trust/summary`

## Run
```bash
cd backend/rust
cargo run                      # CORS allows http://localhost:3000 and :5175
NAVL_SOLVER=grpc cargo run     # use gRPC planner at $PLANNER_ADDR (default 127.0.0.1:50051)
```

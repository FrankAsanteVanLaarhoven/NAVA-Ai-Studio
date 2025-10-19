/**
 * VNC Code Templates
 * 
 * Parameterized templates for common VNC patterns:
 * - REST APIs with CRUD
 * - UI Components (React-style in VNC)
 * - Navigation Algorithms
 * - Database Models
 * - Deployment Configurations
 */

export interface TemplateParams {
  projectName: string;
  entityName?: string;
  fields?: { name: string; type: string }[];
  endpoints?: string[];
  [key: string]: any;
}

export class VNCTemplates {
  /**
   * REST API Template with CRUD operations
   */
  static restAPI(params: TemplateParams): string {
    const { projectName, entityName = 'Item' } = params;
    
    return `// REST API Template for ${projectName}
// Complete CRUD operations with authentication

import { Router⋋, Request⋋, Response⋋ } from '@navlambda/http'
import { Database⋋ } from './database.vnc'
import { AuthMiddleware⋋ } from './middleware/auth.vnc'

class ${entityName}API⋋ {
  db: Database⋋
  
  // CREATE - Add new ${entityName}
  @route("/api/${entityName.toLowerCase()}")
  @method(POST)
  @middleware(AuthMiddleware⋋)
  fn create⋋(request: Request⋋) -> Response⋋ {
    let body = request.json⋋<Create${entityName}Request>()
    
    // Validate input
    if (!body.is_valid⋋()) {
      return json_response⋋(400, { error: "Invalid input" })
    }
    
    // Save to database
    let ${entityName.toLowerCase()} = this.db.${entityName.toLowerCase()}s.insert⋋(body)
    
    return json_response⋋(201, ${entityName.toLowerCase()})
  }
  
  // READ - Get all ${entityName}s
  @route("/api/${entityName.toLowerCase()}")
  @method(GET)
  fn get_all⋋(request: Request⋋) -> Response⋋ {
    let query = request.query_params⋋()
    let limit = query.get("limit").unwrap_or(100)
    let offset = query.get("offset").unwrap_or(0)
    
    let items = this.db.${entityName.toLowerCase()}s
      .find_all⋋()
      .limit⋋(limit)
      .offset⋋(offset)
    
    return json_response⋋(200, items)
  }
  
  // READ - Get single ${entityName}
  @route("/api/${entityName.toLowerCase()}/:id")
  @method(GET)
  fn get_one⋋(request: Request⋋) -> Response⋋ {
    let id = request.params⋋().get("id")
    
    match this.db.${entityName.toLowerCase()}s.find_by_id⋋(id) {
      Some(item) => json_response⋋(200, item),
      None => json_response⋋(404, { error: "Not found" })
    }
  }
  
  // UPDATE - Modify ${entityName}
  @route("/api/${entityName.toLowerCase()}/:id")
  @method(PUT)
  @middleware(AuthMiddleware⋋)
  fn update⋋(request: Request⋋) -> Response⋋ {
    let id = request.params⋋().get("id")
    let body = request.json⋋<Update${entityName}Request>()
    
    match this.db.${entityName.toLowerCase()}s.update⋋(id, body) {
      Ok(item) => json_response⋋(200, item),
      Err(e) => json_response⋋(500, { error: e.to_string() })
    }
  }
  
  // DELETE - Remove ${entityName}
  @route("/api/${entityName.toLowerCase()}/:id")
  @method(DELETE)
  @middleware(AuthMiddleware⋋)
  fn delete⋋(request: Request⋋) -> Response⋋ {
    let id = request.params⋋().get("id")
    
    match this.db.${entityName.toLowerCase()}s.delete⋋(id) {
      Ok(_) => json_response⋋(204, null),
      Err(e) => json_response⋋(500, { error: e.to_string() })
    }
  }
}

export fn setup_${entityName.toLowerCase()}_routes⋋() -> Router⋋ {
  let api = ${entityName}API⋋::new(Database⋋::connect())
  
  return router⋋()
    .route("/api/${entityName.toLowerCase()}", api.create⋋, api.get_all⋋)
    .route("/api/${entityName.toLowerCase()}/:id", api.get_one⋋, api.update⋋, api.delete⋋)
}
`;
  }

  /**
   * UI Component Template (React-style in VNC)
   */
  static uiComponent(params: TemplateParams): string {
    const { entityName = 'Dashboard' } = params;
    
    return `// UI Component: ${entityName}
// React-style component in Van Laarhoven Navigation Calculus

class ${entityName}⋋ {
  // Component state
  state: {
    data: Array<Any>,
    loading: Bool,
    error: String?
  }
  
  // Props definition
  props: {
    title: String,
    on_action: Function<(), ()>,
    show_3d: Bool
  }
  
  // Lifecycle - Component initialization
  fn init⋋() {
    this.state = {
      data: [],
      loading: false,
      error: None
    }
    this.load_data⋋()
  }
  
  // Load data from API
  async fn load_data⋋() {
    this.state.loading = true
    
    match fetch⋋("/api/data") {
      Ok(response) => {
        this.state.data = response.json⋋()
        this.state.loading = false
      },
      Err(e) => {
        this.state.error = Some(e.to_string())
        this.state.loading = false
      }
    }
  }
  
  // Event handler
  fn handle_click⋋(event: Event⋋) {
    console.log⋋("Clicked!")
    this.props.on_action()
  }
  
  // Render function
  fn render⋋() -> UI⋋ {
    return ui⋋ {
      <div class="${entityName.toLowerCase()}-container">
        <h2>{this.props.title}</h2>
        
        {if this.state.loading {
          <Spinner⋋ />
        } else if this.state.error {
          <ErrorMessage⋋ message={this.state.error.unwrap()} />
        } else {
          <div class="content">
            {this.state.data.map⋋(|item| {
              <Card⋋ key={item.id} data={item} />
            })}
          </div>
        }}
        
        {if this.props.show_3d {
          <NavigationVisualizer3D⋋ data={this.state.data} />
        }}
        
        <button 
          onclick={this.handle_click⋋}
          class="action-btn"
        >
          Take Action
        </button>
      </div>
    }
  }
}

export default ${entityName}⋋
`;
  }

  /**
   * Navigation Algorithm Template
   */
  static navigationAlgorithm(params: TemplateParams): string {
    const { projectName } = params;
    
    return `// Van Laarhoven A⋋ Pathfinding Algorithm
// Advanced navigation with energy optimization and 7D position tracking

class VanLaarhovenAStar⋋ {
  manifold: Manifold⋋
  energy_landscape: EnergyLandscape⋋
  
  // Find optimal path using A⋋ (Van Laarhoven A-star)
  fn find_path⋋(start: Position7D⋋, goal: Position7D⋋) -> Result⋋<Path⋋> {
    let open_set = PriorityQueue⋋::new()
    let came_from = HashMap⋋<Position7D⋋, Position7D⋋>::new()
    
    let g_score = HashMap⋋::new()  // Cost from start
    let f_score = HashMap⋋::new()  // Estimated total cost
    
    g_score.insert⋋(start, 0.0)
    f_score.insert⋋(start, this.heuristic⋋(start, goal))
    
    open_set.push⋋(start, f_score[start])
    
    while !open_set.is_empty⋋() {
      let current = open_set.pop⋋()
      
      // Goal reached
      if current == goal {
        return Ok(this.reconstruct_path⋋(came_from, current))
      }
      
      // Explore neighbors in 7D space
      let neighbors = this.get_neighbors_7d⋋(current)
      
      for neighbor in neighbors {
        let tentative_g = g_score[current] + this.cost⋋(current, neighbor)
        
        if tentative_g < g_score.get(neighbor).unwrap_or(Float::INFINITY) {
          came_from.insert⋋(neighbor, current)
          g_score.insert⋋(neighbor, tentative_g)
          f_score.insert⋋(neighbor, tentative_g + this.heuristic⋋(neighbor, goal))
          
          if !open_set.contains⋋(neighbor) {
            open_set.push⋋(neighbor, f_score[neighbor])
          }
        }
      }
    }
    
    return Err("No path found")
  }
  
  // 7D heuristic function
  fn heuristic⋋(pos: Position7D⋋, goal: Position7D⋋) -> Float {
    let spatial = euclidean_distance⋋(pos.xyz(), goal.xyz())
    let temporal = abs(pos.t - goal.t)
    let goal_diff = abs(pos.G - goal.G)
    let intention_diff = abs(pos.I - goal.I)
    let consciousness_diff = abs(pos.C - goal.C)
    
    return spatial + temporal + goal_diff + intention_diff + consciousness_diff
  }
  
  // Calculate movement cost in 7D
  fn cost⋋(from: Position7D⋋, to: Position7D⋋) -> Float {
    let energy = this.energy_landscape.get_energy⋋(to)
    let distance = euclidean_distance⋋(from.xyz(), to.xyz())
    let time_cost = abs(to.t - from.t)
    
    return energy * distance + time_cost
  }
  
  // Get neighboring positions in 7D space
  fn get_neighbors_7d⋋(pos: Position7D⋋) -> Array<Position7D⋋> {
    let neighbors = []
    let delta = 1.0
    
    // Spatial neighbors (x, y, z)
    for dx in [-delta, 0, delta] {
      for dy in [-delta, 0, delta] {
        for dz in [-delta, 0, delta] {
          if dx == 0 && dy == 0 && dz == 0 { continue }
          
          neighbors.push(Position7D⋋(
            pos.t + 0.1,  // Small time step
            pos.x + dx,
            pos.y + dy,
            pos.z + dz,
            0,  // t_offset
            pos.G,  // Goal-directedness preserved
            pos.I,  // Intention preserved
            pos.C   // Consciousness preserved
          ))
        }
      }
    }
    
    return neighbors
  }
  
  // Reconstruct path from came_from map
  fn reconstruct_path⋋(came_from: HashMap⋋, current: Position7D⋋) -> Path⋋ {
    let path = Path⋋::new()
    path.add⋋(current)
    
    while came_from.contains_key⋋(current) {
      current = came_from[current]
      path.prepend⋋(current)
    }
    
    return path
  }
}

// Multi-agent coordination
class FleetCoordinator⋋ {
  agents: Array<Agent⋋>
  
  // Coordinate multiple agents avoiding collisions
  fn coordinate_fleet⋋(goals: Array<Position7D⋋>) -> Array<Path⋋> {
    let paths = []
    let reserved_space = SpatialHash⋋::new()
    
    for (i, goal) in goals.enumerate() {
      let agent = this.agents[i]
      let path = this.find_collision_free_path⋋(
        agent.position,
        goal,
        reserved_space
      )
      
      // Reserve space along this path
      for waypoint in path.waypoints {
        reserved_space.insert⋋(waypoint, agent.id)
      }
      
      paths.push(path)
    }
    
    return paths
  }
  
  // Find path that avoids other agents
  fn find_collision_free_path⋋(
    start: Position7D⋋, 
    goal: Position7D⋋,
    reserved: SpatialHash⋋
  ) -> Path⋋ {
    let pathfinder = VanLaarhovenAStar⋋::new()
    
    return pathfinder.find_path⋋(start, goal)
      .with_collision_avoidance⋋(reserved)
      .optimize_energy⋋()
  }
}

export { VanLaarhovenAStar⋋, FleetCoordinator⋋ }
`;
  }

  /**
   * Database Model Template
   */
  static databaseModel(params: TemplateParams): string {
    const { entityName = 'Entity', fields = [] } = params;
    
    const fieldDefinitions = fields.length > 0
      ? fields.map(f => `  ${f.name}: ${f.type}`).join(',\n')
      : `  id: UUID⋋,\n  name: String,\n  created_at: DateTime⋋`;
    
    return `// Database Model: ${entityName}
// Entity definition with relationships and query builders

@entity
@table("${entityName.toLowerCase()}s")
class ${entityName}⋋ {
${fieldDefinitions}
}

// Repository for database operations
class ${entityName}Repository⋋ {
  db: Database⋋
  
  // Find all entities
  async fn find_all⋋() -> Result⋋<Array<${entityName}⋋>> {
    return this.db.query⋋("SELECT * FROM ${entityName.toLowerCase()}s")
  }
  
  // Find by ID
  async fn find_by_id⋋(id: UUID⋋) -> Result⋋<${entityName}⋋?> {
    return this.db.query_one⋋(
      "SELECT * FROM ${entityName.toLowerCase()}s WHERE id = $1",
      [id]
    )
  }
  
  // Create new entity
  async fn create⋋(entity: ${entityName}⋋) -> Result⋋<${entityName}⋋> {
    return this.db.insert⋋("${entityName.toLowerCase()}s", entity)
  }
  
  // Update entity
  async fn update⋋(id: UUID⋋, updates: Partial<${entityName}⋋>) -> Result⋋<${entityName}⋋> {
    return this.db.update⋋("${entityName.toLowerCase()}s", id, updates)
  }
  
  // Delete entity
  async fn delete⋋(id: UUID⋋) -> Result⋋<()> {
    return this.db.delete⋋("${entityName.toLowerCase()}s", id)
  }
}

// Migration
@migration("create_${entityName.toLowerCase()}s_table")
fn create_table⋋() -> String {
  return """
    CREATE TABLE ${entityName.toLowerCase()}s (
      id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
      ${fields.map(f => `${f.name} ${this.vncTypeToSQL(f.type)}`).join(',\n      ')},
      created_at TIMESTAMPTZ DEFAULT NOW(),
      updated_at TIMESTAMPTZ DEFAULT NOW()
    );
    
    CREATE INDEX idx_${entityName.toLowerCase()}s_created ON ${entityName.toLowerCase()}s(created_at);
  """
}

export { ${entityName}⋋, ${entityName}Repository⋋ }
`;
  }

  /**
   * Deployment Template (Dockerfile + K8s + CI/CD)
   */
  static deployment(params: TemplateParams): string {
    const { projectName } = params;
    
    return {
      dockerfile: `# Optimized Dockerfile for VNC Project: ${projectName}

# Stage 1: Build
FROM rust:1.75-alpine AS builder

WORKDIR /build

# Install VNC compiler
RUN wget https://github.com/navlambda/vnc-compiler/releases/latest/vnc-compiler \\
    && chmod +x vnc-compiler \\
    && mv vnc-compiler /usr/local/bin/

# Copy VNC source
COPY . .

# Compile VNC to Rust
RUN vnc-compiler compile \\
    --target=rust \\
    --optimize=3 \\
    --output=./target

# Build Rust binary
RUN cargo build --release

# Stage 2: Runtime
FROM alpine:3.19

RUN apk add --no-cache \\
    ca-certificates \\
    libgcc

COPY --from=builder /build/target/release/${projectName} /usr/local/bin/app

# Health check
HEALTHCHECK --interval=30s --timeout=3s \\
  CMD wget --no-verbose --tries=1 --spider http://localhost:8080/health || exit 1

EXPOSE 8080

USER 1000:1000

CMD ["app"]
`,
      dockerCompose: `version: '3.8'

services:
  ${projectName}:
    build:
      context: .
      dockerfile: Dockerfile
    ports:
      - "8080:8080"
    environment:
      - DATABASE_URL=\${DATABASE_URL}
      - REDIS_URL=redis://redis:6379
      - LOG_LEVEL=info
    depends_on:
      db:
        condition: service_healthy
      redis:
        condition: service_started
    restart: unless-stopped

  db:
    image: postgres:16-alpine
    environment:
      - POSTGRES_DB=${projectName}
      - POSTGRES_USER=\${DB_USER:-postgres}
      - POSTGRES_PASSWORD=\${DB_PASSWORD:-password}
    volumes:
      - postgres_data:/var/lib/postgresql/data
      - ./migrations:/docker-entrypoint-initdb.d
    ports:
      - "5432:5432"
    healthcheck:
      test: ["CMD-SHELL", "pg_isready -U postgres"]
      interval: 10s
      timeout: 5s
      retries: 5

  redis:
    image: redis:7-alpine
    ports:
      - "6379:6379"
    volumes:
      - redis_data:/data

volumes:
  postgres_data:
  redis_data:
`,
      kubernetes: `apiVersion: apps/v1
kind: Deployment
metadata:
  name: ${projectName}
  labels:
    app: ${projectName}
    version: v1
spec:
  replicas: 3
  strategy:
    type: RollingUpdate
    rollingUpdate:
      maxSurge: 1
      maxUnavailable: 0
  selector:
    matchLabels:
      app: ${projectName}
  template:
    metadata:
      labels:
        app: ${projectName}
        version: v1
    spec:
      containers:
      - name: app
        image: ${projectName}:latest
        ports:
        - containerPort: 8080
          name: http
        env:
        - name: DATABASE_URL
          valueFrom:
            secretKeyRef:
              name: ${projectName}-secrets
              key: database-url
        - name: REDIS_URL
          value: "redis://redis-service:6379"
        resources:
          requests:
            memory: "256Mi"
            cpu: "250m"
          limits:
            memory: "512Mi"
            cpu: "500m"
        livenessProbe:
          httpGet:
            path: /health
            port: 8080
          initialDelaySeconds: 30
          periodSeconds: 10
        readinessProbe:
          httpGet:
            path: /ready
            port: 8080
          initialDelaySeconds: 5
          periodSeconds: 5
`,
      cicd: `name: Deploy ${projectName}

on:
  push:
    branches: [main]
  pull_request:
    branches: [main]

env:
  REGISTRY: ghcr.io
  IMAGE_NAME: \${{ github.repository }}

jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      
      - name: Install VNC Compiler
        run: |
          wget https://github.com/navlambda/vnc-compiler/releases/latest/vnc-compiler
          chmod +x vnc-compiler
          sudo mv vnc-compiler /usr/local/bin/
      
      - name: Compile VNC to Rust
        run: vnc-compiler compile --target=rust
      
      - name: Run tests
        run: cargo test
  
  build:
    needs: test
    runs-on: ubuntu-latest
    permissions:
      contents: read
      packages: write
    steps:
      - uses: actions/checkout@v4
      
      - name: Build and push Docker image
        uses: docker/build-push-action@v5
        with:
          context: .
          push: true
          tags: \${{ env.REGISTRY }}/\${{ env.IMAGE_NAME }}:latest
  
  deploy:
    needs: build
    runs-on: ubuntu-latest
    if: github.ref == 'refs/heads/main'
    steps:
      - uses: actions/checkout@v4
      
      - name: Deploy to Kubernetes
        run: |
          kubectl apply -f k8s/
          kubectl rollout status deployment/${projectName}
`
    };
  }

  /**
   * Get template by name
   */
  static getTemplate(name: string, params: TemplateParams): string {
    switch (name) {
      case 'rest-api':
        return this.restAPI(params);
      case 'ui-component':
        return this.uiComponent(params);
      case 'navigation-algorithm':
        return this.navigationAlgorithm(params);
      case 'database-model':
        return this.databaseModel(params);
      default:
        return this.restAPI(params);
    }
  }

  /**
   * Get all available templates
   */
  static listTemplates(): { name: string; description: string; category: string }[] {
    return [
      { name: 'rest-api', description: 'Complete REST API with CRUD operations', category: 'Backend' },
      { name: 'ui-component', description: 'React-style UI component in VNC', category: 'Frontend' },
      { name: 'navigation-algorithm', description: 'Van Laarhoven A⋋ pathfinding', category: 'Navigation' },
      { name: 'database-model', description: 'Entity with repository pattern', category: 'Database' },
      { name: 'websocket-server', description: 'Real-time WebSocket server', category: 'Backend' },
      { name: '3d-visualizer', description: '3D navigation visualization', category: 'Frontend' },
      { name: 'deployment-full', description: 'Complete deployment stack', category: 'DevOps' },
    ];
  }

  /**
   * Helper: Convert VNC type to SQL type
   */
  private static vncTypeToSQL(vncType: string): string {
    const typeMap: Record<string, string> = {
      'String': 'TEXT',
      'Int': 'INTEGER',
      'Float': 'DOUBLE PRECISION',
      'Bool': 'BOOLEAN',
      'UUID⋋': 'UUID',
      'DateTime⋋': 'TIMESTAMPTZ',
      'Position7D⋋': 'JSONB',
    };
    
    return typeMap[vncType] || 'TEXT';
  }
}

export default VNCTemplates;


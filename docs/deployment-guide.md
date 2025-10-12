# NAVΛ Studio Deployment Guide

Complete guide for deploying NAVΛ applications to production.

## Deployment Options

### 1. Docker Deployment

#### Generate Dockerfile

```bash
navλ compile --target docker your-app.vnc
```

Generated Dockerfile:
```dockerfile
FROM rust:1.75 as builder

WORKDIR /app
COPY . .
RUN cargo build --release

FROM debian:bookworm-slim
COPY --from=builder /app/target/release/app /usr/local/bin/

EXPOSE 8080
CMD ["app"]
```

#### Build and Run

```bash
# Build container
docker build -t navλ-app:latest .

# Run locally
docker run -p 8080:8080 navλ-app:latest

# Push to registry
docker tag navλ-app:latest registry.example.com/navλ-app:latest
docker push registry.example.com/navλ-app:latest
```

### 2. Kubernetes Deployment

#### Generate Kubernetes Manifests

```bash
navλ compile --target kubernetes your-app.vnc
```

#### Deploy to Cluster

```bash
# Apply deployment
kubectl apply -f deployment.yaml

# Check status
kubectl get pods
kubectl logs -f pod-name

# Expose service
kubectl expose deployment navλ-app --type=LoadBalancer --port=8080
```

#### Auto-Scaling

```yaml
apiVersion: autoscaling/v2
kind: HorizontalPodAutoscaler
metadata:
  name: navλ-app-hpa
spec:
  scaleTargetRef:
    apiVersion: apps/v1
    kind: Deployment
    name: navλ-app
  minReplicas: 2
  maxReplicas: 10
  metrics:
  - type: Resource
    resource:
      name: cpu
      target:
        type: Utilization
        averageUtilization: 70
```

### 3. Cloud Platforms

#### AWS Deployment

```bash
# Build for AWS Lambda
navλ compile --target lambda your-app.vnc

# Deploy with AWS SAM
sam build
sam deploy --guided
```

#### Google Cloud Run

```bash
# Build for Cloud Run
gcloud builds submit --tag gcr.io/PROJECT_ID/navλ-app

# Deploy
gcloud run deploy navλ-app \
  --image gcr.io/PROJECT_ID/navλ-app \
  --platform managed \
  --region us-central1 \
  --allow-unauthenticated
```

#### Azure Container Instances

```bash
# Build and push to ACR
az acr build --registry myregistry --image navλ-app:latest .

# Deploy to ACI
az container create \
  --resource-group myResourceGroup \
  --name navλ-app \
  --image myregistry.azurecr.io/navλ-app:latest \
  --cpu 2 \
  --memory 4 \
  --port 8080
```

### 4. Serverless Deployment

#### AWS Lambda

```yaml
# serverless.yml
service: navλ-app

provider:
  name: aws
  runtime: rust
  
functions:
  navigate:
    handler: bootstrap
    events:
      - http:
          path: /navigate
          method: post
```

```bash
# Deploy
serverless deploy
```

#### Cloudflare Workers

```bash
# Build for Workers
navλ compile --target cloudflare-workers your-app.vnc

# Deploy
wrangler publish
```

## CI/CD Integration

### GitHub Actions

```yaml
name: Build and Deploy

on:
  push:
    branches: [main]

jobs:
  build-and-deploy:
    runs-on: ubuntu-latest
    
    steps:
    - uses: actions/checkout@v3
    
    - name: Set up Rust
      uses: actions-rs/toolchain@v1
      with:
        toolchain: stable
        
    - name: Compile NAVΛ app
      run: navλ compile --all-targets your-app.vnc
      
    - name: Build Docker image
      run: docker build -t navλ-app:${{ github.sha }} .
      
    - name: Push to registry
      run: |
        docker push registry.example.com/navλ-app:${{ github.sha }}
        
    - name: Deploy to Kubernetes
      run: |
        kubectl set image deployment/navλ-app \
          app=registry.example.com/navλ-app:${{ github.sha }}
```

### GitLab CI/CD

```yaml
stages:
  - build
  - test
  - deploy

build:
  stage: build
  script:
    - navλ compile --target cpp your-app.vnc
    - cargo build --release
  artifacts:
    paths:
      - target/release/app

deploy:
  stage: deploy
  script:
    - kubectl apply -f k8s/
  only:
    - main
```

## Monitoring and Observability

### Prometheus Metrics

```rust
// Add metrics to your NAVΛ app
use prometheus::{Counter, Registry};

let navigation_counter = Counter::new(
    "navigation_operations_total",
    "Total VNC navigation operations"
)?;

registry.register(Box::new(navigation_counter.clone()))?;
```

### Grafana Dashboard

```json
{
  "dashboard": {
    "title": "NAVΛ Application Metrics",
    "panels": [
      {
        "title": "Navigation Operations",
        "targets": [
          {
            "expr": "rate(navigation_operations_total[5m])"
          }
        ]
      }
    ]
  }
}
```

### Logging

```rust
use tracing::{info, error};

info!(
    navigation_path = ?path,
    energy = path.energy,
    "Navigation completed"
);
```

## Performance Tuning

### Resource Limits

```yaml
apiVersion: v1
kind: Pod
spec:
  containers:
  - name: navλ-app
    resources:
      requests:
        memory: "256Mi"
        cpu: "250m"
      limits:
        memory: "512Mi"
        cpu: "500m"
```

### Caching

```yaml
# Redis cache for navigation results
apiVersion: v1
kind: Service
metadata:
  name: redis
spec:
  ports:
  - port: 6379
  selector:
    app: redis
```

### Database Configuration

```yaml
# PostgreSQL for persistent storage
apiVersion: apps/v1
kind: Deployment
metadata:
  name: postgres
spec:
  replicas: 1
  template:
    spec:
      containers:
      - name: postgres
        image: postgres:15
        env:
        - name: POSTGRES_DB
          value: navlambda
```

## Security

### Secrets Management

```bash
# Kubernetes secrets
kubectl create secret generic navλ-secrets \
  --from-literal=api-key=your-api-key \
  --from-literal=db-password=your-password
```

```yaml
# Use in deployment
env:
- name: API_KEY
  valueFrom:
    secretKeyRef:
      name: navλ-secrets
      key: api-key
```

### Network Policies

```yaml
apiVersion: networking.k8s.io/v1
kind: NetworkPolicy
metadata:
  name: navλ-app-policy
spec:
  podSelector:
    matchLabels:
      app: navλ-app
  policyTypes:
  - Ingress
  - Egress
  ingress:
  - from:
    - podSelector:
        matchLabels:
          role: frontend
    ports:
    - protocol: TCP
      port: 8080
```

### TLS/SSL

```yaml
apiVersion: networking.k8s.io/v1
kind: Ingress
metadata:
  name: navλ-app-ingress
  annotations:
    cert-manager.io/cluster-issuer: letsencrypt-prod
spec:
  tls:
  - hosts:
    - navλ.example.com
    secretName: navλ-tls
  rules:
  - host: navλ.example.com
    http:
      paths:
      - path: /
        pathType: Prefix
        backend:
          service:
            name: navλ-app
            port:
              number: 8080
```

## Backup and Recovery

### Database Backups

```bash
# Automated backup script
#!/bin/bash
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
kubectl exec postgres-0 -- pg_dump -U postgres navlambda > backup_$TIMESTAMP.sql

# Upload to S3
aws s3 cp backup_$TIMESTAMP.sql s3://navλ-backups/
```

### Disaster Recovery

```yaml
# Velero backup
apiVersion: velero.io/v1
kind: Backup
metadata:
  name: navλ-backup
spec:
  includedNamespaces:
  - navλ-production
  storageLocation: default
  volumeSnapshotLocations:
  - default
```

## Health Checks

### Liveness Probe

```yaml
livenessProbe:
  httpGet:
    path: /healthz
    port: 8080
  initialDelaySeconds: 30
  periodSeconds: 10
```

### Readiness Probe

```yaml
readinessProbe:
  httpGet:
    path: /ready
    port: 8080
  initialDelaySeconds: 5
  periodSeconds: 5
```

## Cost Optimization

### Resource Right-Sizing

```bash
# Analyze resource usage
kubectl top pods
kubectl top nodes

# Adjust based on metrics
```

### Auto-Scaling

```yaml
# Cluster autoscaler
apiVersion: autoscaling.k8s.io/v1
kind: ClusterAutoscaler
spec:
  minNodes: 2
  maxNodes: 10
  scaleDownDelay: 10m
```

## Rollback Strategies

### Blue-Green Deployment

```bash
# Deploy new version (green)
kubectl apply -f deployment-green.yaml

# Switch traffic
kubectl patch service navλ-app -p '{"spec":{"selector":{"version":"green"}}}'

# Rollback if needed
kubectl patch service navλ-app -p '{"spec":{"selector":{"version":"blue"}}}'
```

### Canary Deployment

```yaml
# Istio VirtualService for canary
apiVersion: networking.istio.io/v1beta1
kind: VirtualService
metadata:
  name: navλ-app
spec:
  hosts:
  - navλ-app
  http:
  - match:
    - headers:
        canary:
          exact: "true"
    route:
    - destination:
        host: navλ-app
        subset: v2
  - route:
    - destination:
        host: navλ-app
        subset: v1
      weight: 90
    - destination:
        host: navλ-app
        subset: v2
      weight: 10
```

## Troubleshooting

### Common Issues

**Issue**: Pod crashes with OOMKilled

**Solution**: Increase memory limits
```yaml
resources:
  limits:
    memory: "1Gi"
```

**Issue**: Slow navigation performance

**Solution**: Enable VNC optimizations
```bash
navλ compile --vnc-optimize --opt release app.vnc
```

**Issue**: Connection timeouts

**Solution**: Adjust timeouts and retries
```yaml
readinessProbe:
  timeoutSeconds: 5
  failureThreshold: 3
```

## Production Checklist

- [ ] Resource limits configured
- [ ] Health checks implemented
- [ ] Monitoring and alerting set up
- [ ] Logging configured
- [ ] Backups automated
- [ ] Security policies applied
- [ ] TLS/SSL certificates configured
- [ ] Auto-scaling enabled
- [ ] CI/CD pipeline tested
- [ ] Disaster recovery plan documented
- [ ] Performance benchmarks run
- [ ] Load testing completed

---

**Deploy with Confidence**: NAVΛ Studio production-ready 🚀⋋☁️


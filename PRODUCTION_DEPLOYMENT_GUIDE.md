# NAVA Studio Production Deployment Guide

## Overview
This guide provides step-by-step instructions for deploying NAVA Studio to production environments, including cloud infrastructure setup, containerization, monitoring, and scaling strategies.

## Table of Contents
1. [Prerequisites](#prerequisites)
2. [Infrastructure Setup](#infrastructure-setup)
3. [Containerization](#containerization)
4. [Kubernetes Deployment](#kubernetes-deployment)
5. [Monitoring and Logging](#monitoring-and-logging)
6. [Security Configuration](#security-configuration)
7. [Performance Tuning](#performance-tuning)
8. [Disaster Recovery](#disaster-recovery)
9. [Scaling Strategies](#scaling-strategies)

---

## Prerequisites

### Required Tools
```bash
# Install required tools
brew install kubectl helm terraform docker
brew install --cask docker

# Verify installations
kubectl version --client
helm version
terraform version
docker --version
```

### Cloud Provider Setup

#### AWS Setup
```bash
# Install AWS CLI
brew install awscli

# Configure AWS credentials
aws configure
# AWS Access Key ID: YOUR_ACCESS_KEY
# AWS Secret Access Key: YOUR_SECRET_KEY
# Default region: us-west-2
# Default output format: json

# Verify configuration
aws sts get-caller-identity
```

#### GCP Setup
```bash
# Install Google Cloud SDK
brew install --cask google-cloud-sdk

# Initialize gcloud
gcloud init

# Set project
gcloud config set project YOUR_PROJECT_ID

# Authenticate
gcloud auth application-default login
```

### GPU Drivers and CUDA
```bash
# For NVIDIA GPUs
# Download and install CUDA Toolkit 12.0+
# https://developer.nvidia.com/cuda-downloads

# Verify CUDA installation
nvcc --version
nvidia-smi
```

---

## Infrastructure Setup

### Terraform Configuration

Create `infrastructure/main.tf`:
```hcl
terraform {
  required_providers {
    aws = {
      source  = "hashicorp/aws"
      version = "~> 5.0"
    }
  }
}

provider "aws" {
  region = var.aws_region
}

# VPC Configuration
resource "aws_vpc" "nava_vpc" {
  cidr_block           = "10.0.0.0/16"
  enable_dns_hostnames = true
  enable_dns_support   = true

  tags = {
    Name = "nava-studio-vpc"
  }
}

# EKS Cluster
resource "aws_eks_cluster" "nava_cluster" {
  name     = "nava-studio-cluster"
  role_arn = aws_iam_role.eks_cluster_role.arn

  vpc_config {
    subnet_ids = aws_subnet.nava_subnets[*].id
  }

  depends_on = [
    aws_iam_role_policy_attachment.eks_cluster_policy,
  ]
}

# Node Group with GPU instances
resource "aws_eks_node_group" "nava_gpu_nodes" {
  cluster_name    = aws_eks_cluster.nava_cluster.name
  node_group_name = "nava-gpu-nodes"
  node_role_arn   = aws_iam_role.eks_node_role.arn
  subnet_ids      = aws_subnet.nava_subnets[*].id

  instance_types = ["g4dn.xlarge"] # NVIDIA T4 GPU

  scaling_config {
    desired_size = 3
    max_size     = 10
    min_size     = 1
  }

  tags = {
    Name = "nava-gpu-nodes"
  }
}

# S3 Bucket for datasets
resource "aws_s3_bucket" "nava_datasets" {
  bucket = "nava-studio-datasets"

  tags = {
    Name = "nava-datasets"
  }
}

# ElastiCache for Redis (caching)
resource "aws_elasticache_cluster" "nava_cache" {
  cluster_id           = "nava-cache"
  engine               = "redis"
  node_type            = "cache.r6g.large"
  num_cache_nodes      = 1
  parameter_group_name = "default.redis7"
  port                 = 6379
}

# RDS for PostgreSQL (metadata storage)
resource "aws_db_instance" "nava_db" {
  identifier           = "nava-studio-db"
  engine               = "postgres"
  engine_version       = "15.3"
  instance_class       = "db.t3.large"
  allocated_storage    = 100
  storage_type         = "gp3"
  db_name              = "nava_studio"
  username             = var.db_username
  password             = var.db_password
  skip_final_snapshot  = false
  final_snapshot_identifier = "nava-studio-final-snapshot"

  tags = {
    Name = "nava-studio-db"
  }
}
```

Create `infrastructure/variables.tf`:
```hcl
variable "aws_region" {
  description = "AWS region"
  type        = string
  default     = "us-west-2"
}

variable "db_username" {
  description = "Database username"
  type        = string
  sensitive   = true
}

variable "db_password" {
  description = "Database password"
  type        = string
  sensitive   = true
}
```

### Deploy Infrastructure
```bash
cd infrastructure

# Initialize Terraform
terraform init

# Plan deployment
terraform plan -out=tfplan

# Apply configuration
terraform apply tfplan

# Get cluster credentials
aws eks update-kubeconfig --name nava-studio-cluster --region us-west-2
```

---

## Containerization

### Dockerfile for Backend
Create `Dockerfile.backend`:
```dockerfile
# Multi-stage build for Rust backend
FROM rust:1.75 as builder

WORKDIR /app

# Copy manifests
COPY Cargo.toml Cargo.lock ./
COPY src-tauri ./src-tauri

# Build dependencies (cached layer)
RUN cargo build --release --manifest-path src-tauri/Cargo.toml

# Build application
RUN cargo build --release --manifest-path src-tauri/Cargo.toml

# Runtime stage
FROM ubuntu:22.04

# Install runtime dependencies
RUN apt-get update && apt-get install -y \
    libssl3 \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Copy binary from builder
COPY --from=builder /app/target/release/nava-studio /usr/local/bin/

# Expose port
EXPOSE 8080

# Run application
CMD ["nava-studio"]
```

### Dockerfile for Frontend
Create `Dockerfile.frontend`:
```dockerfile
# Build stage
FROM node:20-alpine as builder

WORKDIR /app

# Copy package files
COPY package*.json ./

# Install dependencies
RUN npm ci

# Copy source
COPY . .

# Build application
RUN npm run build

# Production stage
FROM nginx:alpine

# Copy built files
COPY --from=builder /app/dist /usr/share/nginx/html

# Copy nginx configuration
COPY nginx.conf /etc/nginx/nginx.conf

EXPOSE 80

CMD ["nginx", "-g", "daemon off;"]
```

### Docker Compose for Local Development
Create `docker-compose.yml`:
```yaml
version: '3.8'

services:
  backend:
    build:
      context: .
      dockerfile: Dockerfile.backend
    ports:
      - "8080:8080"
    environment:
      - RUST_LOG=info
      - DATABASE_URL=postgresql://postgres:password@db:5432/nava_studio
      - REDIS_URL=redis://redis:6379
    depends_on:
      - db
      - redis
    volumes:
      - ./data:/data

  frontend:
    build:
      context: .
      dockerfile: Dockerfile.frontend
    ports:
      - "3000:80"
    depends_on:
      - backend

  db:
    image: postgres:15
    environment:
      - POSTGRES_DB=nava_studio
      - POSTGRES_USER=postgres
      - POSTGRES_PASSWORD=password
    volumes:
      - postgres_data:/var/lib/postgresql/data
    ports:
      - "5432:5432"

  redis:
    image: redis:7-alpine
    ports:
      - "6379:6379"
    volumes:
      - redis_data:/data

  kafka:
    image: confluentinc/cp-kafka:7.5.0
    ports:
      - "9092:9092"
    environment:
      - KAFKA_BROKER_ID=1
      - KAFKA_ZOOKEEPER_CONNECT=zookeeper:2181
      - KAFKA_ADVERTISED_LISTENERS=PLAINTEXT://kafka:9092
    depends_on:
      - zookeeper

  zookeeper:
    image: confluentinc/cp-zookeeper:7.5.0
    ports:
      - "2181:2181"
    environment:
      - ZOOKEEPER_CLIENT_PORT=2181

volumes:
  postgres_data:
  redis_data:
```

### Build and Push Images
```bash
# Build images
docker build -t nava-studio-backend:latest -f Dockerfile.backend .
docker build -t nava-studio-frontend:latest -f Dockerfile.frontend .

# Tag for registry
docker tag nava-studio-backend:latest YOUR_REGISTRY/nava-studio-backend:latest
docker tag nava-studio-frontend:latest YOUR_REGISTRY/nava-studio-frontend:latest

# Push to registry
docker push YOUR_REGISTRY/nava-studio-backend:latest
docker push YOUR_REGISTRY/nava-studio-frontend:latest
```

---

## Kubernetes Deployment

### Namespace Configuration
Create `k8s/namespace.yaml`:
```yaml
apiVersion: v1
kind: Namespace
metadata:
  name: nava-studio
```

### Backend Deployment
Create `k8s/backend-deployment.yaml`:
```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: nava-backend
  namespace: nava-studio
spec:
  replicas: 3
  selector:
    matchLabels:
      app: nava-backend
  template:
    metadata:
      labels:
        app: nava-backend
    spec:
      containers:
      - name: backend
        image: YOUR_REGISTRY/nava-studio-backend:latest
        ports:
        - containerPort: 8080
        resources:
          requests:
            memory: "4Gi"
            cpu: "2"
            nvidia.com/gpu: 1
          limits:
            memory: "8Gi"
            cpu: "4"
            nvidia.com/gpu: 1
        env:
        - name: RUST_LOG
          value: "info"
        - name: DATABASE_URL
          valueFrom:
            secretKeyRef:
              name: nava-secrets
              key: database-url
        - name: REDIS_URL
          valueFrom:
            secretKeyRef:
              name: nava-secrets
              key: redis-url
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
          initialDelaySeconds: 10
          periodSeconds: 5
---
apiVersion: v1
kind: Service
metadata:
  name: nava-backend-service
  namespace: nava-studio
spec:
  selector:
    app: nava-backend
  ports:
  - protocol: TCP
    port: 80
    targetPort: 8080
  type: LoadBalancer
```

### Frontend Deployment
Create `k8s/frontend-deployment.yaml`:
```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: nava-frontend
  namespace: nava-studio
spec:
  replicas: 2
  selector:
    matchLabels:
      app: nava-frontend
  template:
    metadata:
      labels:
        app: nava-frontend
    spec:
      containers:
      - name: frontend
        image: YOUR_REGISTRY/nava-studio-frontend:latest
        ports:
        - containerPort: 80
        resources:
          requests:
            memory: "512Mi"
            cpu: "500m"
          limits:
            memory: "1Gi"
            cpu: "1"
---
apiVersion: v1
kind: Service
metadata:
  name: nava-frontend-service
  namespace: nava-studio
spec:
  selector:
    app: nava-frontend
  ports:
  - protocol: TCP
    port: 80
    targetPort: 80
  type: LoadBalancer
```

### Horizontal Pod Autoscaler
Create `k8s/hpa.yaml`:
```yaml
apiVersion: autoscaling/v2
kind: HorizontalPodAutoscaler
metadata:
  name: nava-backend-hpa
  namespace: nava-studio
spec:
  scaleTargetRef:
    apiVersion: apps/v1
    kind: Deployment
    name: nava-backend
  minReplicas: 3
  maxReplicas: 10
  metrics:
  - type: Resource
    resource:
      name: cpu
      target:
        type: Utilization
        averageUtilization: 70
  - type: Resource
    resource:
      name: memory
      target:
        type: Utilization
        averageUtilization: 80
```

### Secrets Management
Create `k8s/secrets.yaml`:
```yaml
apiVersion: v1
kind: Secret
metadata:
  name: nava-secrets
  namespace: nava-studio
type: Opaque
stringData:
  database-url: "postgresql://user:password@host:5432/nava_studio"
  redis-url: "redis://redis-host:6379"
  api-key: "your-api-key"
```

### Deploy to Kubernetes
```bash
# Create namespace
kubectl apply -f k8s/namespace.yaml

# Create secrets
kubectl apply -f k8s/secrets.yaml

# Deploy backend
kubectl apply -f k8s/backend-deployment.yaml

# Deploy frontend
kubectl apply -f k8s/frontend-deployment.yaml

# Deploy autoscaler
kubectl apply -f k8s/hpa.yaml

# Verify deployments
kubectl get pods -n nava-studio
kubectl get services -n nava-studio
```

---

## Monitoring and Logging

### Prometheus Setup
Create `k8s/prometheus.yaml`:
```yaml
apiVersion: v1
kind: ConfigMap
metadata:
  name: prometheus-config
  namespace: nava-studio
data:
  prometheus.yml: |
    global:
      scrape_interval: 15s
    scrape_configs:
      - job_name: 'nava-backend'
        kubernetes_sd_configs:
          - role: pod
            namespaces:
              names:
                - nava-studio
        relabel_configs:
          - source_labels: [__meta_kubernetes_pod_label_app]
            action: keep
            regex: nava-backend
---
apiVersion: apps/v1
kind: Deployment
metadata:
  name: prometheus
  namespace: nava-studio
spec:
  replicas: 1
  selector:
    matchLabels:
      app: prometheus
  template:
    metadata:
      labels:
        app: prometheus
    spec:
      containers:
      - name: prometheus
        image: prom/prometheus:latest
        ports:
        - containerPort: 9090
        volumeMounts:
        - name: config
          mountPath: /etc/prometheus
      volumes:
      - name: config
        configMap:
          name: prometheus-config
---
apiVersion: v1
kind: Service
metadata:
  name: prometheus
  namespace: nava-studio
spec:
  selector:
    app: prometheus
  ports:
  - port: 9090
    targetPort: 9090
```

### Grafana Setup
```bash
# Install Grafana using Helm
helm repo add grafana https://grafana.github.io/helm-charts
helm repo update

helm install grafana grafana/grafana \
  --namespace nava-studio \
  --set adminPassword=admin \
  --set service.type=LoadBalancer

# Get Grafana URL
kubectl get svc -n nava-studio grafana
```

### ELK Stack for Logging
Create `k8s/elasticsearch.yaml`:
```yaml
apiVersion: apps/v1
kind: StatefulSet
metadata:
  name: elasticsearch
  namespace: nava-studio
spec:
  serviceName: elasticsearch
  replicas: 3
  selector:
    matchLabels:
      app: elasticsearch
  template:
    metadata:
      labels:
        app: elasticsearch
    spec:
      containers:
      - name: elasticsearch
        image: docker.elastic.co/elasticsearch/elasticsearch:8.11.0
        ports:
        - containerPort: 9200
        - containerPort: 9300
        env:
        - name: discovery.type
          value: "zen"
        - name: ES_JAVA_OPTS
          value: "-Xms2g -Xmx2g"
        volumeMounts:
        - name: data
          mountPath: /usr/share/elasticsearch/data
  volumeClaimTemplates:
  - metadata:
      name: data
    spec:
      accessModes: [ "ReadWriteOnce" ]
      resources:
        requests:
          storage: 100Gi
```

---

## Security Configuration

### TLS/SSL Setup
```bash
# Install cert-manager
kubectl apply -f https://github.com/cert-manager/cert-manager/releases/download/v1.13.0/cert-manager.yaml

# Create ClusterIssuer
cat <<EOF | kubectl apply -f -
apiVersion: cert-manager.io/v1
kind: ClusterIssuer
metadata:
  name: letsencrypt-prod
spec:
  acme:
    server: https://acme-v02.api.letsencrypt.org/directory
    email: your-email@example.com
    privateKeySecretRef:
      name: letsencrypt-prod
    solvers:
    - http01:
        ingress:
          class: nginx
EOF
```

### Network Policies
Create `k8s/network-policy.yaml`:
```yaml
apiVersion: networking.k8s.io/v1
kind: NetworkPolicy
metadata:
  name: nava-backend-policy
  namespace: nava-studio
spec:
  podSelector:
    matchLabels:
      app: nava-backend
  policyTypes:
  - Ingress
  - Egress
  ingress:
  - from:
    - podSelector:
        matchLabels:
          app: nava-frontend
    ports:
    - protocol: TCP
      port: 8080
  egress:
  - to:
    - podSelector:
        matchLabels:
          app: postgres
    ports:
    - protocol: TCP
      port: 5432
```

---

## Performance Tuning

### Resource Optimization
```yaml
# Optimize resource requests/limits
resources:
  requests:
    memory: "4Gi"
    cpu: "2"
  limits:
    memory: "8Gi"
    cpu: "4"
```

### Caching Strategy
```rust
// Implement Redis caching
use redis::AsyncCommands;

pub async fn get_cached_data(key: &str) -> Option<String> {
    let client = redis::Client::open("redis://redis:6379").ok()?;
    let mut con = client.get_async_connection().await.ok()?;
    con.get(key).await.ok()
}
```

---

## Disaster Recovery

### Backup Strategy
```bash
# Automated database backups
kubectl create cronjob nava-db-backup \
  --image=postgres:15 \
  --schedule="0 2 * * *" \
  --namespace=nava-studio \
  -- /bin/sh -c "pg_dump -h db-host -U user nava_studio > /backups/backup-$(date +%Y%m%d).sql"
```

### Restore Procedure
```bash
# Restore from backup
kubectl exec -it postgres-pod -n nava-studio -- \
  psql -U user -d nava_studio < /backups/backup-20250101.sql
```

---

## Scaling Strategies

### Vertical Scaling
```bash
# Increase pod resources
kubectl set resources deployment nava-backend \
  --requests=cpu=4,memory=8Gi \
  --limits=cpu=8,memory=16Gi \
  -n nava-studio
```

### Horizontal Scaling
```bash
# Manual scaling
kubectl scale deployment nava-backend --replicas=10 -n nava-studio

# Auto-scaling is handled by HPA
```

---

## Production Checklist

- [ ] Infrastructure provisioned with Terraform
- [ ] Docker images built and pushed to registry
- [ ] Kubernetes cluster configured
- [ ] All deployments running successfully
- [ ] Monitoring and logging set up
- [ ] TLS/SSL certificates configured
- [ ] Network policies applied
- [ ] Backup strategy implemented
- [ ] Auto-scaling configured
- [ ] Load testing completed
- [ ] Security audit passed
- [ ] Documentation updated

---

**Document Version**: 1.0  
**Last Updated**: 2025  
**Status**: Production Ready

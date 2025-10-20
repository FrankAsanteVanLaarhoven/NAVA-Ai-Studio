# NAVΛ Studio Cloud Deployment Guide

## Overview

This guide provides comprehensive instructions for deploying NAVΛ Studio to a cloud environment using Kubernetes, Docker, and modern DevOps practices.

## Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Frontend      │    │   Backend       │    │   Kafka         │
│   (React)       │◄──►│   (Rust/Tauri)  │◄──►│   (Data Pipeline)│
│                 │    │                 │    │                 │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         └───────────────────────┼───────────────────────┘
                                 │
                    ┌─────────────────┐
                    │   Monitoring    │
                    │ (Prometheus/Grafana)│
                    └─────────────────┘
```

## Prerequisites

### Required Tools
- **Docker** 20.10+
- **kubectl** 1.24+
- **Helm** 3.8+
- **Kubernetes cluster** (EKS, GKE, AKS, or self-managed)

### Cluster Requirements
- **Nodes**: At least 3 worker nodes
- **CPU**: 4+ vCPUs per node
- **Memory**: 8GB+ RAM per node
- **Storage**: 100GB+ SSD storage
- **GPU**: NVIDIA GPUs for AI workloads (optional but recommended)

## Quick Start

### 1. Clone and Setup
```bash
git clone <repository-url>
cd navlambda-studio
```

### 2. Configure Environment
```bash
# Update secrets in k8s/secrets.yaml
# Update ingress domain in k8s/ingress.yaml
# Configure your container registry in deploy.sh
```

### 3. Deploy Everything
```bash
./deploy.sh all
```

## Detailed Deployment

### Step-by-Step Deployment

#### 1. Prerequisites Check
```bash
./deploy.sh check
```

#### 2. Build Docker Images
```bash
./deploy.sh build
```

#### 3. Push to Registry
```bash
./deploy.sh push
```

#### 4. Deploy to Kubernetes
```bash
./deploy.sh deploy
```

#### 5. Deploy Monitoring
```bash
./deploy.sh monitoring
```

#### 6. Check Status
```bash
./deploy.sh status
```

## Configuration

### Environment Variables

#### Backend Configuration
```yaml
env:
- name: RUST_LOG
  value: "info"
- name: DATABASE_URL
  valueFrom:
    secretKeyRef:
      name: navlambda-secrets
      key: database-url
- name: REDIS_URL
  valueFrom:
    secretKeyRef:
      name: navlambda-secrets
      key: redis-url
- name: KAFKA_BROKERS
  value: "navlambda-kafka:9092"
```

#### Frontend Configuration
```yaml
env:
- name: REACT_APP_API_URL
  value: "http://navlambda-backend:8080"
```

### Secrets Management

Update `k8s/secrets.yaml` with your actual secrets:

```yaml
apiVersion: v1
kind: Secret
metadata:
  name: navlambda-secrets
type: Opaque
data:
  database-url: <base64-encoded-postgres-url>
  redis-url: <base64-encoded-redis-url>
  kafka-password: <base64-encoded-password>
  jwt-secret: <base64-encoded-jwt-secret>
```

### Storage Configuration

The deployment includes persistent volume claims for:
- **Models Storage**: 100GB for AI models
- **Data Storage**: 500GB for datasets and user data
- **Kafka Storage**: 50GB for message queue data

## Monitoring and Observability

### Prometheus Metrics
- CPU/Memory utilization
- Request latency and throughput
- Custom NAVΛ Studio metrics

### Grafana Dashboards
- System overview
- Application performance
- AI model metrics
- Data pipeline monitoring

### Access Monitoring
```bash
# Get Grafana admin password
kubectl get secret --namespace navlambda-studio grafana -o jsonpath="{.data.admin-password}" | base64 --decode

# Port forward Grafana
kubectl port-forward svc/grafana 3000:80 -n navlambda-studio

# Access at http://localhost:3000
```

## Scaling

### Horizontal Pod Autoscaling
The deployment includes HPAs for:
- **Backend**: Scales based on CPU (70%), Memory (80%), and custom metrics
- **Frontend**: Scales based on CPU utilization

### Manual Scaling
```bash
# Scale backend to 5 replicas
kubectl scale deployment navlambda-backend --replicas=5 -n navlambda-studio

# Scale frontend to 3 replicas
kubectl scale deployment navlambda-frontend --replicas=3 -n navlambda-studio
```

## Troubleshooting

### Common Issues

#### Pods Not Starting
```bash
# Check pod status
kubectl get pods -n navlambda-studio

# Check pod logs
kubectl logs <pod-name> -n navlambda-studio

# Describe pod for events
kubectl describe pod <pod-name> -n navlambda-studio
```

#### Image Pull Errors
```bash
# Check if images exist in registry
docker pull navlambda/backend:latest
docker pull navlambda/frontend:latest

# Verify registry credentials
kubectl get secrets -n navlambda-studio
```

#### Service Connectivity
```bash
# Test service connectivity
kubectl exec -it <pod-name> -n navlambda-studio -- curl http://navlambda-backend:8080/health

# Check service endpoints
kubectl get endpoints -n navlambda-studio
```

### Logs and Debugging

#### Application Logs
```bash
# Backend logs
kubectl logs -f deployment/navlambda-backend -n navlambda-studio

# Frontend logs
kubectl logs -f deployment/navlambda-frontend -n navlambda-studio

# Kafka logs
kubectl logs -f deployment/navlambda-kafka -n navlambda-studio
```

#### System Logs
```bash
# Node logs
kubectl logs -f <node-name> --namespace kube-system

# Ingress controller logs
kubectl logs -f deployment/nginx-ingress-controller -n ingress-nginx
```

## Security

### Network Policies
```yaml
apiVersion: networking.k8s.io/v1
kind: NetworkPolicy
metadata:
  name: navlambda-network-policy
  namespace: navlambda-studio
spec:
  podSelector: {}
  policyTypes:
  - Ingress
  - Egress
  ingress:
  - from:
    - namespaceSelector:
        matchLabels:
          name: ingress-nginx
    ports:
    - protocol: TCP
      port: 80
    - protocol: TCP
      port: 8080
```

### TLS Configuration
The deployment includes automatic TLS certificate management using cert-manager and Let's Encrypt.

## Backup and Recovery

### Database Backup
```bash
# Create database backup
kubectl exec -it <postgres-pod> -n navlambda-studio -- pg_dump -U navlambda navlambda > backup.sql

# Restore from backup
kubectl exec -it <postgres-pod> -n navlambda-studio -- psql -U navlambda navlambda < backup.sql
```

### PVC Backup
```bash
# Backup PVC data
kubectl cp navlambda-studio/<pod-name>:/app/models ./models-backup
kubectl cp navlambda-studio/<pod-name>:/app/data ./data-backup
```

## Performance Optimization

### Resource Limits
```yaml
resources:
  requests:
    cpu: 1000m
    memory: 2Gi
    nvidia.com/gpu: 1
  limits:
    cpu: 2000m
    memory: 4Gi
    nvidia.com/gpu: 1
```

### Node Affinity
```yaml
nodeSelector:
  accelerator: nvidia-tesla-k80
tolerations:
- key: "nvidia.com/gpu"
  operator: "Exists"
  effect: "NoSchedule"
```

## CI/CD Integration

### GitHub Actions
The deployment script integrates with GitHub Actions for automated deployments:

```yaml
- name: Deploy to Kubernetes
  run: |
    ./deploy.sh build
    ./deploy.sh push
    ./deploy.sh deploy
```

## Cost Optimization

### Spot Instances
```yaml
nodeSelector:
  lifecycle: spot
tolerations:
- key: "lifecycle"
  operator: "Equal"
  value: "spot"
  effect: "NoSchedule"
```

### Auto-scaling Policies
Configure cluster autoscaler for automatic node scaling based on workload demands.

## Support

### Documentation
- [Kubernetes Documentation](https://kubernetes.io/docs/)
- [Helm Documentation](https://helm.sh/docs/)
- [Docker Documentation](https://docs.docker.com/)

### Community
- [NAVΛ Studio GitHub](https://github.com/navlambda/navlambda-studio)
- [Kubernetes Slack](https://slack.k8s.io/)
- [Docker Community](https://forums.docker.com/)

## License

This deployment configuration is part of NAVΛ Studio and is licensed under the MIT License.
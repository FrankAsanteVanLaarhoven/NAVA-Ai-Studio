#!/bin/bash

# NAVΛ Studio Cloud Deployment Script
set -e

echo "🚀 Starting NAVΛ Studio Cloud Deployment"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check prerequisites
check_prerequisites() {
    print_status "Checking prerequisites..."

    # Check if Docker is installed
    if ! command -v docker &> /dev/null; then
        print_error "Docker is not installed. Please install Docker first."
        exit 1
    fi

    # Check if kubectl is installed
    if ! command -v kubectl &> /dev/null; then
        print_error "kubectl is not installed. Please install kubectl first."
        exit 1
    fi

    # Check if helm is installed
    if ! command -v helm &> /dev/null; then
        print_error "Helm is not installed. Please install Helm first."
        exit 1
    fi

    # Check if cluster is accessible
    if ! kubectl cluster-info &> /dev/null; then
        print_error "Cannot access Kubernetes cluster. Please configure kubectl."
        exit 1
    fi

    print_success "Prerequisites check passed"
}

# Build Docker images
build_images() {
    print_status "Building Docker images..."

    # Build backend image
    print_status "Building backend image..."
    docker build -f Dockerfile.backend -t navlambda/backend:latest .

    # Build frontend image
    print_status "Building frontend image..."
    docker build -f Dockerfile.frontend -t navlambda/frontend:latest .

    print_success "Docker images built successfully"
}

# Push images to registry
push_images() {
    print_status "Pushing images to registry..."

    # Tag and push backend image
    docker tag navlambda/backend:latest navlambda/backend:$(date +%Y%m%d-%H%M%S)
    docker push navlambda/backend:latest

    # Tag and push frontend image
    docker tag navlambda/frontend:latest navlambda/frontend:$(date +%Y%m%d-%H%M%S)
    docker push navlambda/frontend:latest

    print_success "Images pushed to registry"
}

# Deploy to Kubernetes
deploy_kubernetes() {
    print_status "Deploying to Kubernetes..."

    # Create namespace
    print_status "Creating namespace..."
    kubectl apply -f k8s/namespace.yaml

    # Deploy persistent volume claims
    print_status "Creating persistent volume claims..."
    kubectl apply -f k8s/pvc.yaml

    # Deploy secrets
    print_status "Creating secrets..."
    kubectl apply -f k8s/secrets.yaml

    # Deploy Kafka
    print_status "Deploying Kafka..."
    kubectl apply -f k8s/kafka-deployment.yaml

    # Wait for Kafka to be ready
    print_status "Waiting for Kafka to be ready..."
    kubectl wait --for=condition=available --timeout=300s deployment/navlambda-kafka -n navlambda-studio
    kubectl wait --for=condition=available --timeout=300s deployment/navlambda-zookeeper -n navlambda-studio

    # Deploy backend
    print_status "Deploying backend..."
    kubectl apply -f k8s/backend-deployment.yaml
    kubectl apply -f k8s/backend-service.yaml

    # Deploy frontend
    print_status "Deploying frontend..."
    kubectl apply -f k8s/frontend-deployment.yaml
    kubectl apply -f k8s/frontend-service.yaml

    # Deploy ingress
    print_status "Deploying ingress..."
    kubectl apply -f k8s/ingress.yaml

    # Deploy autoscaling
    print_status "Setting up autoscaling..."
    kubectl apply -f k8s/hpa.yaml

    print_success "Kubernetes deployment completed"
}

# Deploy monitoring stack
deploy_monitoring() {
    print_status "Deploying monitoring stack..."

    # Add Helm repos
    print_status "Adding Helm repositories..."
    helm repo add prometheus-community https://prometheus-community.github.io/helm-charts
    helm repo add grafana https://grafana.github.io/helm-charts
    helm repo update

    # Install Prometheus
    print_status "Installing Prometheus..."
    helm upgrade --install prometheus prometheus-community/prometheus \
        --namespace navlambda-studio \
        --create-namespace \
        --set server.persistentVolume.enabled=true \
        --set server.persistentVolume.size=50Gi

    # Install Grafana
    print_status "Installing Grafana..."
    helm upgrade --install grafana grafana/grafana \
        --namespace navlambda-studio \
        --set adminPassword=admin \
        --set persistence.enabled=true \
        --set persistence.size=10Gi

    print_success "Monitoring stack deployed"
}

# Wait for deployments to be ready
wait_for_deployments() {
    print_status "Waiting for deployments to be ready..."

    # Wait for backend
    print_status "Waiting for backend..."
    kubectl wait --for=condition=available --timeout=300s deployment/navlambda-backend -n navlambda-studio

    # Wait for frontend
    print_status "Waiting for frontend..."
    kubectl wait --for=condition=available --timeout=300s deployment/navlambda-frontend -n navlambda-studio

    print_success "All deployments are ready"
}

# Get deployment status
get_status() {
    print_status "Getting deployment status..."

    echo ""
    echo "📊 Deployment Status:"
    echo "===================="

    echo ""
    echo "Namespaces:"
    kubectl get namespaces | grep navlambda

    echo ""
    echo "Pods:"
    kubectl get pods -n navlambda-studio

    echo ""
    echo "Services:"
    kubectl get services -n navlambda-studio

    echo ""
    echo "Ingress:"
    kubectl get ingress -n navlambda-studio

    echo ""
    echo "Persistent Volumes:"
    kubectl get pvc -n navlambda-studio

    echo ""
    echo "Horizontal Pod Autoscalers:"
    kubectl get hpa -n navlambda-studio

    # Get Grafana admin password
    GRAFANA_PASSWORD=$(kubectl get secret --namespace navlambda-studio grafana -o jsonpath="{.data.admin-password}" | base64 --decode ; echo)
    echo ""
    echo "🔐 Grafana Credentials:"
    echo "URL: http://grafana.navlambda-studio.svc.cluster.local"
    echo "Username: admin"
    echo "Password: $GRAFANA_PASSWORD"
}

# Main deployment function
main() {
    echo "🎯 NAVΛ Studio Cloud Deployment"
    echo "================================"

    case "${1:-all}" in
        "check")
            check_prerequisites
            ;;
        "build")
            check_prerequisites
            build_images
            ;;
        "push")
            check_prerequisites
            push_images
            ;;
        "deploy")
            check_prerequisites
            deploy_kubernetes
            ;;
        "monitoring")
            check_prerequisites
            deploy_monitoring
            ;;
        "status")
            get_status
            ;;
        "all")
            check_prerequisites
            build_images
            push_images
            deploy_kubernetes
            deploy_monitoring
            wait_for_deployments
            get_status
            ;;
        *)
            echo "Usage: $0 {check|build|push|deploy|monitoring|status|all}"
            echo ""
            echo "Commands:"
            echo "  check      - Check prerequisites"
            echo "  build      - Build Docker images"
            echo "  push       - Push images to registry"
            echo "  deploy     - Deploy to Kubernetes"
            echo "  monitoring - Deploy monitoring stack"
            echo "  status     - Show deployment status"
            echo "  all        - Run complete deployment"
            exit 1
            ;;
    esac

    print_success "Deployment completed successfully! 🎉"
    echo ""
    echo "Next steps:"
    echo "1. Update your DNS to point to the ingress IP"
    echo "2. Access the application at https://navlambda.example.com"
    echo "3. Access Grafana at https://grafana.navlambda.example.com"
    echo "4. Monitor logs with: kubectl logs -f deployment/navlambda-backend -n navlambda-studio"
}

# Run main function with all arguments
main "$@"
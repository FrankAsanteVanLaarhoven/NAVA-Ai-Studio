use std::collections::HashMap;
use std::process::Command;
use serde::{Deserialize, Serialize};
use tokio::process::Command as AsyncCommand;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct DeploymentConfig {
    pub cluster_name: String,
    pub region: String,
    pub node_groups: Vec<NodeGroup>,
    pub services: Vec<ServiceConfig>,
    pub ingress: IngressConfig,
    pub monitoring: MonitoringConfig,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct NodeGroup {
    pub name: String,
    pub instance_type: String,
    pub min_size: i32,
    pub max_size: i32,
    pub desired_size: i32,
    pub gpu_enabled: bool,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ServiceConfig {
    pub name: String,
    pub image: String,
    pub replicas: i32,
    pub ports: Vec<i32>,
    pub env_vars: HashMap<String, String>,
    pub resources: ResourceRequirements,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ResourceRequirements {
    pub cpu: String,
    pub memory: String,
    pub gpu: Option<String>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct IngressConfig {
    pub domain: String,
    pub tls_enabled: bool,
    pub rules: Vec<IngressRule>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct IngressRule {
    pub host: String,
    pub paths: Vec<String>,
    pub service: String,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct MonitoringConfig {
    pub prometheus_enabled: bool,
    pub grafana_enabled: bool,
    pub elk_enabled: bool,
    pub custom_dashboards: Vec<String>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct DeploymentStatus {
    pub cluster_status: String,
    pub services_status: HashMap<String, String>,
    pub ingress_status: String,
    pub monitoring_status: String,
    pub last_updated: String,
}

pub struct CloudDeployer {
    config: DeploymentConfig,
    kubectl_available: bool,
    helm_available: bool,
    docker_available: bool,
}

impl CloudDeployer {
    pub fn new(config: DeploymentConfig) -> Self {
        CloudDeployer {
            config,
            kubectl_available: false,
            helm_available: false,
            docker_available: false,
        }
    }

    pub async fn initialize(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        // Check if required tools are available
        self.kubectl_available = Self::check_command("kubectl").await;
        self.helm_available = Self::check_command("helm").await;
        self.docker_available = Self::check_command("docker").await;

        if !self.kubectl_available {
            return Err("kubectl not found. Please install kubectl.".into());
        }
        if !self.helm_available {
            return Err("helm not found. Please install Helm.".into());
        }
        if !self.docker_available {
            return Err("docker not found. Please install Docker.".into());
        }

        println!("Cloud deployment tools verified successfully");
        Ok(())
    }

    async fn check_command(command: &str) -> bool {
        AsyncCommand::new(command)
            .arg("--version")
            .output()
            .await
            .map(|output| output.status.success())
            .unwrap_or(false)
    }

    pub async fn create_namespace(&self) -> Result<(), Box<dyn std::error::Error>> {
        let namespace = "navlambda-studio";

        let output = AsyncCommand::new("kubectl")
            .args(&["create", "namespace", namespace, "--dry-run=client", "-o", "yaml"])
            .output()
            .await?;

        if output.status.success() {
            // Apply the namespace
            AsyncCommand::new("kubectl")
                .args(&["create", "namespace", namespace])
                .output()
                .await?;
        }

        println!("Namespace '{}' created or already exists", namespace);
        Ok(())
    }

    pub async fn deploy_services(&self) -> Result<(), Box<dyn std::error::Error>> {
        for service in &self.config.services {
            self.deploy_service(service).await?;
        }
        Ok(())
    }

    async fn deploy_service(&self, service: &ServiceConfig) -> Result<(), Box<dyn std::error::Error>> {
        // Create deployment YAML
        let deployment_yaml = self.generate_deployment_yaml(service)?;

        // Apply the deployment
        let mut apply_cmd = AsyncCommand::new("kubectl");
        apply_cmd.args(&["apply", "-f", "-"]);

        let mut child = apply_cmd
            .stdin(std::process::Stdio::piped())
            .spawn()?;

        if let Some(stdin) = &mut child.stdin {
            use std::io::Write;
            stdin.write_all(deployment_yaml.as_bytes())?;
        }

        let output = child.wait_with_output().await?;
        if !output.status.success() {
            return Err(format!("Failed to deploy service {}: {}",
                service.name,
                String::from_utf8_lossy(&output.stderr)
            ).into());
        }

        // Create service YAML
        let service_yaml = self.generate_service_yaml(service)?;

        // Apply the service
        let mut apply_svc_cmd = AsyncCommand::new("kubectl");
        apply_svc_cmd.args(&["apply", "-f", "-"]);

        let mut svc_child = apply_svc_cmd
            .stdin(std::process::Stdio::piped())
            .spawn()?;

        if let Some(stdin) = &mut svc_child.stdin {
            use std::io::Write;
            stdin.write_all(service_yaml.as_bytes())?;
        }

        let svc_output = svc_child.wait_with_output().await?;
        if !svc_output.status.success() {
            return Err(format!("Failed to create service {}: {}",
                service.name,
                String::from_utf8_lossy(&svc_output.stderr)
            ).into());
        }

        println!("Service '{}' deployed successfully", service.name);
        Ok(())
    }

    pub async fn deploy_ingress(&self) -> Result<(), Box<dyn std::error::Error>> {
        let ingress_yaml = self.generate_ingress_yaml()?;

        let mut apply_cmd = AsyncCommand::new("kubectl");
        apply_cmd.args(&["apply", "-f", "-"]);

        let mut child = apply_cmd
            .stdin(std::process::Stdio::piped())
            .spawn()?;

        if let Some(stdin) = &mut child.stdin {
            use std::io::Write;
            stdin.write_all(ingress_yaml.as_bytes())?;
        }

        let output = child.wait_with_output().await?;
        if !output.status.success() {
            return Err(format!("Failed to deploy ingress: {}",
                String::from_utf8_lossy(&output.stderr)
            ).into());
        }

        println!("Ingress deployed successfully");
        Ok(())
    }

    pub async fn deploy_monitoring(&self) -> Result<(), Box<dyn std::error::Error>> {
        if self.config.monitoring.prometheus_enabled {
            self.deploy_prometheus().await?;
        }

        if self.config.monitoring.grafana_enabled {
            self.deploy_grafana().await?;
        }

        if self.config.monitoring.elk_enabled {
            self.deploy_elk().await?;
        }

        Ok(())
    }

    async fn deploy_prometheus(&self) -> Result<(), Box<dyn std::error::Error>> {
        // Add Prometheus Helm repo
        AsyncCommand::new("helm")
            .args(&["repo", "add", "prometheus-community", "https://prometheus-community.github.io/helm-charts"])
            .output()
            .await?;

        AsyncCommand::new("helm")
            .args(&["repo", "update"])
            .output()
            .await?;

        // Install Prometheus
        AsyncCommand::new("helm")
            .args(&[
                "upgrade", "--install", "prometheus", "prometheus-community/prometheus",
                "--namespace", "navlambda-studio",
                "--create-namespace"
            ])
            .output()
            .await?;

        println!("Prometheus deployed successfully");
        Ok(())
    }

    async fn deploy_grafana(&self) -> Result<(), Box<dyn std::error::Error>> {
        // Add Grafana Helm repo
        AsyncCommand::new("helm")
            .args(&["repo", "add", "grafana", "https://grafana.github.io/helm-charts"])
            .output()
            .await?;

        AsyncCommand::new("helm")
            .args(&["repo", "update"])
            .output()
            .await?;

        // Install Grafana
        AsyncCommand::new("helm")
            .args(&[
                "upgrade", "--install", "grafana", "grafana/grafana",
                "--namespace", "navlambda-studio",
                "--set", "adminPassword=admin"
            ])
            .output()
            .await?;

        println!("Grafana deployed successfully");
        Ok(())
    }

    async fn deploy_elk(&self) -> Result<(), Box<dyn std::error::Error>> {
        // Deploy Elasticsearch
        let es_yaml = r#"
apiVersion: apps/v1
kind: Deployment
metadata:
  name: elasticsearch
  namespace: navlambda-studio
spec:
  replicas: 1
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
        image: docker.elastic.co/elasticsearch/elasticsearch:8.5.0
        env:
        - name: discovery.type
          value: single-node
        ports:
        - containerPort: 9200
---
apiVersion: v1
kind: Service
metadata:
  name: elasticsearch
  namespace: navlambda-studio
spec:
  selector:
    app: elasticsearch
  ports:
  - port: 9200
    targetPort: 9200
"#;

        AsyncCommand::new("kubectl")
            .args(&["apply", "-f", "-"])
            .stdin(std::process::Stdio::piped())
            .spawn()?
            .stdin
            .as_mut()
            .unwrap()
            .write_all(es_yaml.as_bytes())?;

        println!("ELK stack deployed successfully");
        Ok(())
    }

    pub async fn setup_autoscaling(&self) -> Result<(), Box<dyn std::error::Error>> {
        for service in &self.config.services {
            self.setup_hpa(service).await?;
        }
        Ok(())
    }

    async fn setup_hpa(&self, service: &ServiceConfig) -> Result<(), Box<dyn std::error::Error>> {
        let hpa_yaml = format!(r#"
apiVersion: autoscaling/v2
kind: HorizontalPodAutoscaler
metadata:
  name: {}-hpa
  namespace: navlambda-studio
spec:
  scaleTargetRef:
    apiVersion: apps/v1
    kind: Deployment
    name: {}
  minReplicas: 1
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
"#, service.name, service.name);

        AsyncCommand::new("kubectl")
            .args(&["apply", "-f", "-"])
            .stdin(std::process::Stdio::piped())
            .spawn()?
            .stdin
            .as_mut()
            .unwrap()
            .write_all(hpa_yaml.as_bytes())?;

        println!("HPA configured for service '{}'", service.name);
        Ok(())
    }

    pub async fn get_deployment_status(&self) -> Result<DeploymentStatus, Box<dyn std::error::Error>> {
        // Get cluster status
        let cluster_output = AsyncCommand::new("kubectl")
            .args(&["cluster-info"])
            .output()
            .await?;

        let cluster_status = if cluster_output.status.success() {
            "healthy".to_string()
        } else {
            "unhealthy".to_string()
        };

        // Get services status
        let mut services_status = HashMap::new();
        for service in &self.config.services {
            let svc_output = AsyncCommand::new("kubectl")
                .args(&["get", "pods", "-l", &format!("app={}", service.name), "-n", "navlambda-studio"])
                .output()
                .await?;

            let status = if svc_output.status.success() {
                let output_str = String::from_utf8_lossy(&svc_output.stdout);
                if output_str.contains("Running") {
                    "running".to_string()
                } else {
                    "pending".to_string()
                }
            } else {
                "failed".to_string()
            };

            services_status.insert(service.name.clone(), status);
        }

        // Get ingress status
        let ingress_output = AsyncCommand::new("kubectl")
            .args(&["get", "ingress", "-n", "navlambda-studio"])
            .output()
            .await?;

        let ingress_status = if ingress_output.status.success() {
            "configured".to_string()
        } else {
            "not_configured".to_string()
        };

        let monitoring_status = if self.config.monitoring.prometheus_enabled ||
                                  self.config.monitoring.grafana_enabled ||
                                  self.config.monitoring.elk_enabled {
            "deployed".to_string()
        } else {
            "not_deployed".to_string()
        };

        Ok(DeploymentStatus {
            cluster_status,
            services_status,
            ingress_status,
            monitoring_status,
            last_updated: chrono::Utc::now().to_rfc3339(),
        })
    }

    fn generate_deployment_yaml(&self, service: &ServiceConfig) -> Result<String, Box<dyn std::error::Error>> {
        let env_vars: Vec<String> = service.env_vars.iter()
            .map(|(k, v)| format!("            - name: {}\n              value: \"{}\"", k, v))
            .collect();

        let ports: Vec<String> = service.ports.iter()
            .map(|port| format!("            - containerPort: {}", port))
            .collect();

        let resources = if let Some(gpu) = &service.resources.gpu {
            format!(r#"
          resources:
            requests:
              cpu: {}
              memory: {}
              nvidia.com/gpu: {}
            limits:
              cpu: {}
              memory: {}
              nvidia.com/gpu: {}"#, service.resources.cpu, service.resources.memory, gpu,
              service.resources.cpu, service.resources.memory, gpu)
        } else {
            format!(r#"
          resources:
            requests:
              cpu: {}
              memory: {}
            limits:
              cpu: {}
              memory: {}"#, service.resources.cpu, service.resources.memory,
              service.resources.cpu, service.resources.memory)
        };

        let yaml = format!(r#"apiVersion: apps/v1
kind: Deployment
metadata:
  name: {}
  namespace: navlambda-studio
spec:
  replicas: {}
  selector:
    matchLabels:
      app: {}
  template:
    metadata:
      labels:
        app: {}
    spec:
      containers:
      - name: {}
        image: {}
        ports:
{}
        env:
{}
{}
"#, service.name, service.replicas, service.name, service.name, service.name, service.image,
   ports.join("\n"), env_vars.join("\n"), resources);

        Ok(yaml)
    }

    fn generate_service_yaml(&self, service: &ServiceConfig) -> Result<String, Box<dyn std::error::Error>> {
        let ports: Vec<String> = service.ports.iter()
            .enumerate()
            .map(|(i, port)| format!("      - port: {}\n        targetPort: {}\n        name: port-{}", port, port, i))
            .collect();

        let yaml = format!(r#"apiVersion: v1
kind: Service
metadata:
  name: {}
  namespace: navlambda-studio
spec:
  selector:
    app: {}
  ports:
{}
  type: ClusterIP
"#, service.name, service.name, ports.join("\n"));

        Ok(yaml)
    }

    fn generate_ingress_yaml(&self) -> Result<String, Box<dyn std::error::Error>> {
        let rules: Vec<String> = self.config.ingress.rules.iter()
            .map(|rule| {
                let paths: Vec<String> = rule.paths.iter()
                    .map(|path| format!("            - path: {}\n              pathType: Prefix", path))
                    .collect();

                format!(r#"      - host: {}
        http:
          paths:
{}"#, rule.host, paths.join(""))
            })
            .collect();

        let tls = if self.config.ingress.tls_enabled {
            format!(r#"
  tls:
  - hosts:
    - {}
    secretName: navlambda-tls"#, self.config.ingress.domain)
        } else {
            "".to_string()
        };

        let yaml = format!(r#"apiVersion: networking.k8s.io/v1
kind: Ingress
metadata:
  name: navlambda-ingress
  namespace: navlambda-studio
  annotations:
    nginx.ingress.kubernetes.io/rewrite-target: /
spec:
  rules:
{}
{}"#, rules.join("\n"), tls);

        Ok(yaml)
    }
}

pub async fn deploy_to_cloud() -> Result<String, Box<dyn std::error::Error>> {
    println!("Starting cloud deployment...");

    // Default deployment configuration
    let config = DeploymentConfig {
        cluster_name: "navlambda-cluster".to_string(),
        region: "us-west-2".to_string(),
        node_groups: vec![
            NodeGroup {
                name: "cpu-nodes".to_string(),
                instance_type: "t3.large".to_string(),
                min_size: 2,
                max_size: 10,
                desired_size: 3,
                gpu_enabled: false,
            },
            NodeGroup {
                name: "gpu-nodes".to_string(),
                instance_type: "g4dn.xlarge".to_string(),
                min_size: 0,
                max_size: 5,
                desired_size: 1,
                gpu_enabled: true,
            },
        ],
        services: vec![
            ServiceConfig {
                name: "navlambda-backend".to_string(),
                image: "navlambda/backend:latest".to_string(),
                replicas: 3,
                ports: vec![8080],
                env_vars: HashMap::new(),
                resources: ResourceRequirements {
                    cpu: "1000m".to_string(),
                    memory: "2Gi".to_string(),
                    gpu: Some("1".to_string()),
                },
            },
            ServiceConfig {
                name: "navlambda-frontend".to_string(),
                image: "navlambda/frontend:latest".to_string(),
                replicas: 2,
                ports: vec![80],
                env_vars: HashMap::new(),
                resources: ResourceRequirements {
                    cpu: "500m".to_string(),
                    memory: "512Mi".to_string(),
                    gpu: None,
                },
            },
        ],
        ingress: IngressConfig {
            domain: "navlambda.example.com".to_string(),
            tls_enabled: true,
            rules: vec![
                IngressRule {
                    host: "navlambda.example.com".to_string(),
                    paths: vec!["/".to_string()],
                    service: "navlambda-frontend".to_string(),
                },
                IngressRule {
                    host: "api.navlambda.example.com".to_string(),
                    paths: vec!["/".to_string()],
                    service: "navlambda-backend".to_string(),
                },
            ],
        },
        monitoring: MonitoringConfig {
            prometheus_enabled: true,
            grafana_enabled: true,
            elk_enabled: true,
            custom_dashboards: vec![],
        },
    };

    let mut deployer = CloudDeployer::new(config);
    deployer.initialize().await?;
    deployer.create_namespace().await?;
    deployer.deploy_services().await?;
    deployer.deploy_ingress().await?;
    deployer.deploy_monitoring().await?;
    deployer.setup_autoscaling().await?;

    let status = deployer.get_deployment_status().await?;
    println!("Deployment completed. Status: {:?}", status);

    Ok("Cloud deployment completed successfully".to_string())
}

pub async fn scale_simulation(replicas: i32) -> Result<String, Box<dyn std::error::Error>> {
    println!("Scaling simulation instances to {} replicas", replicas);

    // Scale the backend service
    let output = AsyncCommand::new("kubectl")
        .args(&[
            "scale", "deployment", "navlambda-backend",
            "--replicas", &replicas.to_string(),
            "-n", "navlambda-studio"
        ])
        .output()
        .await?;

    if output.status.success() {
        println!("Successfully scaled simulation to {} replicas", replicas);
        Ok(format!("Simulation scaled to {} replicas", replicas))
    } else {
        Err(format!("Failed to scale simulation: {}",
            String::from_utf8_lossy(&output.stderr)
        ).into())
    }
}

pub async fn get_deployment_status() -> Result<String, Box<dyn std::error::Error>> {
    let config = DeploymentConfig {
        cluster_name: "navlambda-cluster".to_string(),
        region: "us-west-2".to_string(),
        node_groups: vec![],
        services: vec![],
        ingress: IngressConfig {
            domain: "".to_string(),
            tls_enabled: false,
            rules: vec![],
        },
        monitoring: MonitoringConfig {
            prometheus_enabled: false,
            grafana_enabled: false,
            elk_enabled: false,
            custom_dashboards: vec![],
        },
    };

    let deployer = CloudDeployer::new(config);
    let status = deployer.get_deployment_status().await?;
    Ok(serde_json::to_string_pretty(&status)?)
}
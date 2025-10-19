import { EventEmitter } from 'events';
// Use performance from browser environment instead of Node.js perf_hooks
// import { performance, PerformanceObserver } from 'perf_hooks';

export interface PerformanceMetrics {
  compilationTime: number;
  memoryUsage: number;
  cpuUsage: number;
  networkLatency: number;
  renderTime: number;
  userInteractionTime: number;
}

export interface MetricThresholds {
  maxCompilationTime: number;
  maxMemoryUsage: number;
  maxCpuUsage: number;
  maxNetworkLatency: number;
  maxRenderTime: number;
  maxUserInteractionTime: number;
}

export interface PerformanceReport {
  timestamp: number;
  metrics: PerformanceMetrics;
  thresholds: MetricThresholds;
  alerts: PerformanceAlert[];
  recommendations: string[];
}

export interface PerformanceAlert {
  type: 'warning' | 'error' | 'critical';
  metric: keyof PerformanceMetrics;
  value: number;
  threshold: number;
  message: string;
  timestamp: number;
}

export class PerformanceMonitor extends EventEmitter {
  private metrics: PerformanceMetrics;
  private thresholds: MetricThresholds;
  private alerts: PerformanceAlert[] = [];
  private performanceObserver: any; // Use 'any' to avoid browser/node conflicts
  private monitoringInterval: NodeJS.Timeout | null = null;
  private startTime: number = 0;
  private baselineMetrics: PerformanceMetrics | null = null;

  constructor(thresholds: Partial<MetricThresholds> = {}) {
    super();

    this.thresholds = {
      maxCompilationTime: 2000,      // 2 seconds
      maxMemoryUsage: 100 * 1024 * 1024, // 100MB
      maxCpuUsage: 0.8,               // 80%
      maxNetworkLatency: 500,         // 500ms
      maxRenderTime: 100,             // 100ms
      maxUserInteractionTime: 50,     // 50ms
      ...thresholds
    };

    this.metrics = {
      compilationTime: 0,
      memoryUsage: 0,
      cpuUsage: 0,
      networkLatency: 0,
      renderTime: 0,
      userInteractionTime: 0
    };

    this.startTime = Date.now();
    // Skip performance observer setup for now to avoid browser/node conflicts
    // this.setupPerformanceObserver();
  }

  private setupPerformanceObserver(): void {
    // Simplified performance observer for browser environment
    if (typeof window !== 'undefined' && (window as any).PerformanceObserver) {
      this.performanceObserver = new (window as any).PerformanceObserver((list: any) => {
        for (const entry of list.getEntries()) {
          this.processPerformanceEntry(entry);
        }
      });

      this.performanceObserver.observe({
        entryTypes: ['measure', 'navigation', 'resource', 'paint', 'longtask']
      });
    }
  }

  private processPerformanceEntry(entry: any): void {

  private processPerformanceEntry(entry: PerformanceEntry): void {
    switch (entry.entryType) {
      case 'measure':
        this.handleMeasureEntry(entry as PerformanceMeasure);
        break;
      case 'navigation':
        this.handleNavigationEntry(entry as PerformanceNavigationTiming);
        break;
      case 'resource':
        this.handleResourceEntry(entry as PerformanceResourceTiming);
        break;
      case 'paint':
        this.handlePaintEntry(entry as PerformancePaintTiming);
        break;
      case 'longtask':
        this.handleLongTaskEntry(entry as any);
        break;
    }
  }

  private handleMeasureEntry(entry: PerformanceMeasure): void {
    if (entry.name.includes('compilation')) {
      this.metrics.compilationTime = entry.duration;
      this.checkThreshold('compilationTime', entry.duration);
    } else if (entry.name.includes('render')) {
      this.metrics.renderTime = entry.duration;
      this.checkThreshold('renderTime', entry.duration);
    } else if (entry.name.includes('interaction')) {
      this.metrics.userInteractionTime = entry.duration;
      this.checkThreshold('userInteractionTime', entry.duration);
    }
  }

  private handleNavigationEntry(entry: PerformanceNavigationTiming): void {
    this.metrics.networkLatency = entry.responseStart - entry.requestStart;
    this.checkThreshold('networkLatency', this.metrics.networkLatency);
  }

  private handleResourceEntry(entry: PerformanceResourceTiming): void {
    if (entry.duration > this.thresholds.maxNetworkLatency) {
      this.createAlert('warning', 'networkLatency', entry.duration, this.thresholds.maxNetworkLatency,
        `Resource loading took ${entry.duration}ms, exceeding threshold of ${this.thresholds.maxNetworkLatency}ms`);
    }
  }

  private handlePaintEntry(entry: PerformancePaintTiming): void {
    if (entry.name === 'first-contentful-paint') {
      this.emit('firstContentfulPaint', entry.startTime);
    } else if (entry.name === 'largest-contentful-paint') {
      this.emit('largestContentfulPaint', entry.startTime);
    }
  }

  private handleLongTaskEntry(entry: any): void {
    if (entry.duration > 50) {
      this.createAlert('warning', 'userInteractionTime', entry.duration, 50,
        `Long task detected: ${entry.duration}ms`);
    }
  }

  private checkThreshold(metric: keyof PerformanceMetrics, value: number): void {
    let thresholdKey: keyof MetricThresholds;

    // Map metric names to threshold names
    switch (metric) {
      case 'compilationTime':
        thresholdKey = 'maxCompilationTime';
        break;
      case 'memoryUsage':
        thresholdKey = 'maxMemoryUsage';
        break;
      case 'cpuUsage':
        thresholdKey = 'maxCpuUsage';
        break;
      case 'networkLatency':
        thresholdKey = 'maxNetworkLatency';
        break;
      case 'renderTime':
        thresholdKey = 'maxRenderTime';
        break;
      case 'userInteractionTime':
        thresholdKey = 'maxUserInteractionTime';
        break;
      default:
        // If no specific mapping, use the same name with 'max' prefix
        thresholdKey = `max${metric.charAt(0).toUpperCase() + metric.slice(1)}` as keyof MetricThresholds;
    }

    const threshold = this.thresholds[thresholdKey];

    if (value > threshold) {
      const severity = this.determineAlertSeverity(metric, value, threshold);
      const message = `${metric} exceeded threshold: ${value}ms (threshold: ${threshold}ms)`;

      this.createAlert(severity, metric, value, threshold, message);
    }
  }

  private determineAlertSeverity(
    metric: keyof PerformanceMetrics,
    value: number,
    threshold: number
  ): 'warning' | 'error' | 'critical' {
    const ratio = value / threshold;
    
    if (ratio > 2.0) return 'critical';
    if (ratio > 1.5) return 'error';
    return 'warning';
  }

  private createAlert(
    type: 'warning' | 'error' | 'critical',
    metric: keyof PerformanceMetrics,
    value: number,
    threshold: number,
    message: string
  ): void {
    const alert: PerformanceAlert = {
      type,
      metric,
      value,
      threshold,
      message,
      timestamp: Date.now()
    };

    this.alerts.push(alert);
    this.emit('alert', alert);

    if (type === 'critical') {
      this.emit('criticalAlert', alert);
    }
  }

  public startMonitoring(intervalMs: number = 1000): void {
    if (this.monitoringInterval) {
      this.stopMonitoring();
    }

    this.monitoringInterval = setInterval(() => {
      this.collectMetrics();
      this.generateReport();
    }, intervalMs);

    this.emit('monitoringStarted');
  }

  public stopMonitoring(): void {
    if (this.monitoringInterval) {
      clearInterval(this.monitoringInterval);
      this.monitoringInterval = null;
      this.emit('monitoringStopped');
    }
  }

  private collectMetrics(): void {
    // Collect memory usage
    if ('memory' in performance) {
      this.metrics.memoryUsage = (performance as any).memory.usedJSHeapSize;
      this.checkThreshold('memoryUsage', this.metrics.memoryUsage);
    }

    // Collect CPU usage (simplified estimation)
    this.estimateCpuUsage();

    // Emit metrics collected event
    this.emit('metricsCollected', this.metrics);
  }

  private estimateCpuUsage(): void {
    // This is a simplified CPU usage estimation
    // In a real implementation, you might use more sophisticated methods
    const start = performance.now();
    
    // Perform a small computation
    let sum = 0;
    for (let i = 0; i < 100000; i++) {
      sum += Math.sqrt(i);
    }
    
    const end = performance.now();
    const computationTime = end - start;
    
    // Estimate CPU usage based on computation time
    this.metrics.cpuUsage = Math.min(computationTime / 10, 1.0); // Normalize to 0-1
    this.checkThreshold('cpuUsage', this.metrics.cpuUsage * 100); // Convert to percentage
  }

  private generateReport(): PerformanceReport {
    const report: PerformanceReport = {
      timestamp: Date.now(),
      metrics: { ...this.metrics },
      thresholds: { ...this.thresholds },
      alerts: [...this.alerts],
      recommendations: this.generateRecommendations()
    };

    this.emit('reportGenerated', report);
    return report;
  }

  private generateRecommendations(): string[] {
    const recommendations: string[] = [];

    if (this.metrics.compilationTime > this.thresholds.maxCompilationTime) {
      recommendations.push('Consider optimizing compilation process or reducing code complexity');
    }

    if (this.metrics.memoryUsage > this.thresholds.maxMemoryUsage) {
      recommendations.push('Monitor memory usage and consider implementing memory cleanup');
    }

    if (this.metrics.cpuUsage > this.thresholds.maxCpuUsage) {
      recommendations.push('High CPU usage detected - consider optimizing algorithms');
    }

    if (this.metrics.networkLatency > this.thresholds.maxNetworkLatency) {
      recommendations.push('High network latency detected - check network connection');
    }

    if (this.metrics.renderTime > this.thresholds.maxRenderTime) {
      recommendations.push('Slow rendering detected - consider optimizing UI components');
    }

    if (this.metrics.userInteractionTime > this.thresholds.maxUserInteractionTime) {
      recommendations.push('Slow user interactions - consider implementing debouncing or throttling');
    }

    return recommendations;
  }

  public recordCompilationTime(duration: number): void {
    performance.mark('compilation-start');
    performance.mark('compilation-end');
    performance.measure('compilation', 'compilation-start', 'compilation-end');
    
    this.metrics.compilationTime = duration;
    this.checkThreshold('compilationTime', duration);
  }

  public recordUserInteraction(interactionType: string, duration: number): void {
    performance.mark(`${interactionType}-start`);
    performance.mark(`${interactionType}-end`);
    performance.measure(`interaction-${interactionType}`, `${interactionType}-start`, `${interactionType}-end`);
    
    this.metrics.userInteractionTime = duration;
    this.checkThreshold('userInteractionTime', duration);
  }

  public recordRenderTime(duration: number): void {
    performance.mark('render-start');
    performance.mark('render-end');
    performance.measure('render', 'render-start', 'render-end');

    this.metrics.renderTime = duration;
    this.checkThreshold('renderTime', duration);
  }

  public recordMemoryUsage(bytes: number): void {
    this.metrics.memoryUsage = bytes;
    this.checkThreshold('memoryUsage', bytes);
  }

  public recordCpuUsage(usage: number): void {
    this.metrics.cpuUsage = usage;
    this.checkThreshold('cpuUsage', usage);
  }

  public recordNetworkLatency(latency: number): void {
    this.metrics.networkLatency = latency;
    this.checkThreshold('networkLatency', latency);
  }

  public recordUserInteractionTime(duration: number): void {
    this.metrics.userInteractionTime = duration;
    this.checkThreshold('userInteractionTime', duration);
  }

  public setBaseline(): void {
    this.baselineMetrics = { ...this.metrics };
    this.emit('baselineSet', this.baselineMetrics);
  }

  public compareWithBaseline(): { [key: string]: number } {
    if (!this.baselineMetrics) {
      return {};
    }

    const comparisons: { [key: string]: number } = {};
    
    for (const key of Object.keys(this.metrics) as Array<keyof PerformanceMetrics>) {
      const baseline = this.baselineMetrics[key];
      const current = this.metrics[key];
      
      if (baseline > 0) {
        comparisons[key] = ((current - baseline) / baseline) * 100;
      }
    }

    return comparisons;
  }

  public getMetrics(): PerformanceMetrics {
    return { ...this.metrics };
  }

  public getAlerts(): PerformanceAlert[] {
    return [...this.alerts];
  }

  public getThresholds(): MetricThresholds {
    return { ...this.thresholds };
  }

  public updateThresholds(newThresholds: Partial<MetricThresholds>): void {
    this.thresholds = { ...this.thresholds, ...newThresholds };
    this.emit('thresholdsUpdated', this.thresholds);
  }

  public clearAlerts(): void {
    this.alerts = [];
    this.emit('alertsCleared');
  }

  public exportData(): string {
    const data = {
      metrics: this.metrics,
      thresholds: this.thresholds,
      alerts: this.alerts,
      baseline: this.baselineMetrics,
      timestamp: Date.now()
    };

    return JSON.stringify(data, null, 2);
  }

  public importData(jsonData: string): void {
    try {
      const data = JSON.parse(jsonData);
      
      if (data.metrics) {
        this.metrics = data.metrics;
      }
      
      if (data.thresholds) {
        this.thresholds = data.thresholds;
      }
      
      if (data.alerts) {
        this.alerts = data.alerts;
      }
      
      if (data.baseline) {
        this.baselineMetrics = data.baseline;
      }
      
      this.emit('dataImported', data);
    } catch (error) {
      this.emit('importError', error);
    }
  }

  public destroy(): void {
    this.stopMonitoring();
    
    if (this.performanceObserver) {
      this.performanceObserver.disconnect();
    }
    
    this.removeAllListeners();
  }
}

export class PerformanceDashboard {
  private monitor: PerformanceMonitor;
  private container: HTMLElement;
  private charts: Map<string, any> = new Map();
  private updateInterval: NodeJS.Timeout | null = null;

  constructor(monitor: PerformanceMonitor, container: HTMLElement) {
    this.monitor = monitor;
    this.container = container;
    this.setupDashboard();
    this.startUpdates();
  }

  private setupDashboard(): void {
    this.container.innerHTML = `
      <div class="performance-dashboard">
        <div class="dashboard-header">
          <h2>Performance Dashboard</h2>
          <div class="dashboard-controls">
            <button id="start-monitoring" class="btn btn-primary">Start Monitoring</button>
            <button id="stop-monitoring" class="btn btn-secondary">Stop Monitoring</button>
            <button id="export-data" class="btn btn-outline">Export Data</button>
            <button id="clear-alerts" class="btn btn-outline">Clear Alerts</button>
          </div>
        </div>
        
        <div class="metrics-grid">
          <div class="metric-card" data-metric="compilationTime">
            <h3>Compilation Time</h3>
            <div class="metric-value">0ms</div>
            <div class="metric-threshold">Threshold: 2000ms</div>
            <div class="metric-status status-good">Good</div>
          </div>
          
          <div class="metric-card" data-metric="memoryUsage">
            <h3>Memory Usage</h3>
            <div class="metric-value">0MB</div>
            <div class="metric-threshold">Threshold: 100MB</div>
            <div class="metric-status status-good">Good</div>
          </div>
          
          <div class="metric-card" data-metric="cpuUsage">
            <h3>CPU Usage</h3>
            <div class="metric-value">0%</div>
            <div class="metric-threshold">Threshold: 80%</div>
            <div class="metric-status status-good">Good</div>
          </div>
          
          <div class="metric-card" data-metric="networkLatency">
            <h3>Network Latency</h3>
            <div class="metric-value">0ms</div>
            <div class="metric-threshold">Threshold: 500ms</div>
            <div class="metric-status status-good">Good</div>
          </div>
          
          <div class="metric-card" data-metric="renderTime">
            <h3>Render Time</h3>
            <div class="metric-value">0ms</div>
            <div class="metric-threshold">Threshold: 100ms</div>
            <div class="metric-status status-good">Good</div>
          </div>
          
          <div class="metric-card" data-metric="userInteractionTime">
            <h3>Interaction Time</h3>
            <div class="metric-value">0ms</div>
            <div class="metric-threshold">Threshold: 50ms</div>
            <div class="metric-status status-good">Good</div>
          </div>
        </div>
        
        <div class="charts-container">
          <div class="chart-wrapper">
            <h3>Performance Trends</h3>
            <canvas id="performance-chart" width="800" height="300"></canvas>
          </div>
          
          <div class="chart-wrapper">
            <h3>Memory Usage</h3>
            <canvas id="memory-chart" width="400" height="200"></canvas>
          </div>
          
          <div class="chart-wrapper">
            <h3>CPU Usage</h3>
            <canvas id="cpu-chart" width="400" height="200"></canvas>
          </div>
        </div>
        
        <div class="alerts-section">
          <h3>Performance Alerts</h3>
          <div id="alerts-container" class="alerts-container">
            <p class="no-alerts">No alerts</p>
          </div>
        </div>
        
        <div class="recommendations-section">
          <h3>Recommendations</h3>
          <div id="recommendations-container" class="recommendations-container">
            <p class="no-recommendations">No recommendations</p>
          </div>
        </div>
      </div>
    `;

    this.setupEventListeners();
  }

  private setupEventListeners(): void {
    const startButton = this.container.querySelector('#start-monitoring') as HTMLButtonElement;
    const stopButton = this.container.querySelector('#stop-monitoring') as HTMLButtonElement;
    const exportButton = this.container.querySelector('#export-data') as HTMLButtonElement;
    const clearButton = this.container.querySelector('#clear-alerts') as HTMLButtonElement;

    startButton?.addEventListener('click', () => {
      this.monitor.startMonitoring(1000);
    });

    stopButton?.addEventListener('click', () => {
      this.monitor.stopMonitoring();
    });

    exportButton?.addEventListener('click', () => {
      const data = this.monitor.exportData();
      const blob = new Blob([data], { type: 'application/json' });
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url;
      a.download = `performance-data-${Date.now()}.json`;
      a.click();
      URL.revokeObjectURL(url);
    });

    clearButton?.addEventListener('click', () => {
      this.monitor.clearAlerts();
    });

    this.monitor.on('metricsCollected', (metrics) => {
      // First update the metric cards using existing logic
      this.updateMetricCards(metrics);

      // Then ensure displayed values and statuses use the correct threshold mapping
      const thresholds = this.monitor.getThresholds();

      for (const metric in metrics) {
        const valueElement = document.querySelector(`[data-metric="${metric}"] .value`) as HTMLElement | null;
        const statusElement = document.querySelector(`[data-metric="${metric}"] .status`) as HTMLElement | null;

        if (valueElement && statusElement && metric in metrics) {
          const value = metrics[metric];

          // Map metric names to threshold names
          let thresholdKey: keyof MetricThresholds;
          switch (metric) {
            case 'compilationTime':
              thresholdKey = 'maxCompilationTime';
              break;
            case 'memoryUsage':
              thresholdKey = 'maxMemoryUsage';
              break;
            case 'cpuUsage':
              thresholdKey = 'maxCpuUsage';
              break;
            case 'networkLatency':
              thresholdKey = 'maxNetworkLatency';
              break;
            case 'renderTime':
              thresholdKey = 'maxRenderTime';
              break;
            case 'userInteractionTime':
              thresholdKey = 'maxUserInteractionTime';
              break;
            default:
              thresholdKey = (`max${metric.charAt(0).toUpperCase() + metric.slice(1)}`) as keyof MetricThresholds;
          }

          const threshold = thresholds[thresholdKey];

          // Update value display
          if (metric === 'memoryUsage') {
            valueElement.textContent = `${(value / (1024 * 1024)).toFixed(2)}MB`;
          } else if (metric === 'cpuUsage') {
            valueElement.textContent = `${(value * 100).toFixed(1)}%`;
          } else {
            // For timing metrics, show ms with no decimals
            valueElement.textContent = `${Number(value).toFixed(0)}ms`;
          }

          // Update status based on threshold (if threshold is numeric)
          if (typeof threshold === 'number') {
            if (value > threshold) {
              statusElement.textContent = 'Warning';
              statusElement.classList.add('alert');
              statusElement.classList.remove('ok');
            } else {
              statusElement.textContent = 'OK';
              statusElement.classList.remove('alert');
              statusElement.classList.add('ok');
            }
          } else {
            // If no threshold, clear status styles
            statusElement.textContent = '';
            statusElement.classList.remove('alert', 'ok');
          }
        }
      }
    });

    this.monitor.on('alert', (alert) => {
      this.addAlert(alert);
    });

    this.monitor.on('reportGenerated', (report) => {
      this.updateRecommendations(report.recommendations);
    });
  }

  private startUpdates(): void {
    this.updateInterval = setInterval(() => {
      this.updateCharts();
    }, 1000);
  }

  private updateMetricCards(metrics: PerformanceMetrics): void {
    const cards = this.container.querySelectorAll('.metric-card');

    cards.forEach(card => {
      const metric = card.getAttribute('data-metric') as keyof PerformanceMetrics;
      const valueElement = card.querySelector('.metric-value');
      const statusElement = card.querySelector('.metric-status');

      if (valueElement && statusElement && metric in metrics) {
        const value = metrics[metric];
        // Map metric names to threshold names
        const thresholdKey = this.getThresholdKey(metric);
        const threshold = this.monitor.getThresholds()[thresholdKey as keyof MetricThresholds];

        // Update value display
        if (metric === 'memoryUsage') {
          valueElement.textContent = `${(value / (1024 * 1024)).toFixed(2)}MB`;
        } else if (metric === 'cpuUsage') {
          valueElement.textContent = `${(value * 100).toFixed(1)}%`;
        } else {
          valueElement.textContent = `${value.toFixed(0)}ms`;
        }

        // Update status
        const ratio = value / threshold;
        if (ratio > 1.5) {
          statusElement.className = 'metric-status status-critical';
          statusElement.textContent = 'Critical';
        } else if (ratio > 1.0) {
          statusElement.className = 'metric-status status-warning';
          statusElement.textContent = 'Warning';
        } else {
          statusElement.className = 'metric-status status-good';
          statusElement.textContent = 'Good';
        }
      }
    });
  }

  private getThresholdKey(metric: keyof PerformanceMetrics): string {
    switch (metric) {
      case 'compilationTime': return 'maxCompilationTime';
      case 'memoryUsage': return 'maxMemoryUsage';
      case 'cpuUsage': return 'maxCpuUsage';
      case 'networkLatency': return 'maxNetworkLatency';
      case 'renderTime': return 'maxRenderTime';
      case 'userInteractionTime': return 'maxUserInteractionTime';
      default: return `max${metric.charAt(0).toUpperCase()}${metric.slice(1)}`;
    }
  }
        }
        
        // Update status
        const ratio = value / threshold;
        if (ratio > 1.5) {
          statusElement.className = 'metric-status status-critical';
          statusElement.textContent = 'Critical';
        } else if (ratio > 1.0) {
          statusElement.className = 'metric-status status-warning';
          statusElement.textContent = 'Warning';
        } else {
          statusElement.className = 'metric-status status-good';
          statusElement.textContent = 'Good';
        }
      }
    });
  }

  private addAlert(alert: PerformanceAlert): void {
    const alertsContainer = this.container.querySelector('#alerts-container');
    const noAlertsMessage = this.container.querySelector('.no-alerts');
    
    if (noAlertsMessage) {
      noAlertsMessage.remove();
    }
    
    if (alertsContainer) {
      const alertElement = document.createElement('div');
      alertElement.className = `alert alert-${alert.type}`;
      alertElement.innerHTML = `
        <div class="alert-header">
          <span class="alert-type">${alert.type.toUpperCase()}</span>
          <span class="alert-time">${new Date(alert.timestamp).toLocaleTimeString()}</span>
        </div>
        <div class="alert-message">${alert.message}</div>
        <div class="alert-details">
          Metric: ${alert.metric} | Value: ${alert.value.toFixed(2)} | Threshold: ${alert.threshold.toFixed(2)}
        </div>
      `;
      
      alertsContainer.appendChild(alertElement);
      
      // Keep only last 10 alerts
      const alerts = alertsContainer.querySelectorAll('.alert');
      if (alerts.length > 10) {
        alerts[0].remove();
      }
    }
  }

  private updateRecommendations(recommendations: string[]): void {
    const recommendationsContainer = this.container.querySelector('#recommendations-container');
    const noRecommendationsMessage = this.container.querySelector('.no-recommendations');
    
    if (noRecommendationsMessage) {
      noRecommendationsMessage.remove();
    }
    
    if (recommendationsContainer) {
      recommendationsContainer.innerHTML = '';
      
      if (recommendations.length === 0) {
        recommendationsContainer.innerHTML = '<p class="no-recommendations">No recommendations</p>';
      } else {
        recommendations.forEach(recommendation => {
          const recElement = document.createElement('div');
          recElement.className = 'recommendation';
          recElement.innerHTML = `
            <div class="recommendation-icon">•</div>
            <div class="recommendation-text">${recommendation}</div>
          `;
          recommendationsContainer.appendChild(recElement);
        });
      }
    }
  }

  private updateCharts(): void {
    // This would be implemented with a charting library like Chart.js
    // For now, we'll just update the canvas elements
    const performanceCanvas = this.container.querySelector('#performance-chart') as HTMLCanvasElement;
    const memoryCanvas = this.container.querySelector('#memory-chart') as HTMLCanvasElement;
    const cpuCanvas = this.container.querySelector('#cpu-chart') as HTMLCanvasElement;

    if (performanceCanvas && performanceCanvas.getContext) {
      const ctx = performanceCanvas.getContext('2d');
      if (ctx) {
        // Draw simple performance chart
        ctx.clearRect(0, 0, performanceCanvas.width, performanceCanvas.height);
        ctx.fillStyle = '#4CAF50';
        ctx.fillRect(10, 10, 100, 20);
        ctx.fillStyle = '#333';
        ctx.font = '12px Arial';
        ctx.fillText('Performance Data', 120, 25);
      }
    }

    // Similar implementations for memory and CPU charts
    if (memoryCanvas && memoryCanvas.getContext) {
      const ctx = memoryCanvas.getContext('2d');
      if (ctx) {
        ctx.clearRect(0, 0, memoryCanvas.width, memoryCanvas.height);
        ctx.fillStyle = '#2196F3';
        ctx.fillRect(10, 10, 80, 15);
        ctx.fillStyle = '#333';
        ctx.font = '10px Arial';
        ctx.fillText('Memory: 45MB', 100, 22);
      }
    }

    if (cpuCanvas && cpuCanvas.getContext) {
      const ctx = cpuCanvas.getContext('2d');
      if (ctx) {
        ctx.clearRect(0, 0, cpuCanvas.width, cpuCanvas.height);
        ctx.fillStyle = '#FF9800';
        ctx.fillRect(10, 10, 60, 15);
        ctx.fillStyle = '#333';
        ctx.font = '10px Arial';
        ctx.fillText('CPU: 25%', 80, 22);
      }
    }
  }

  public destroy(): void {
    if (this.updateInterval) {
      clearInterval(this.updateInterval);
    }
    
    this.monitor.off('metricsCollected', this.updateMetricCards);
    this.monitor.off('alert', this.addAlert);
    this.monitor.off('reportGenerated', this.updateRecommendations);
  }
}

export class PerformanceAnalytics {
  private monitor: PerformanceMonitor;
  private data: PerformanceReport[] = [];
  private sessionStart: number;

  constructor(monitor: PerformanceMonitor) {
    this.monitor = monitor;
    this.sessionStart = Date.now();
    this.setupAnalytics();
  }

  private setupAnalytics(): void {
    this.monitor.on('reportGenerated', (report) => {
      this.data.push(report);
      this.analyzeTrends();
    });
  }

  private analyzeTrends(): void {
    if (this.data.length < 5) return;

    const recentData = this.data.slice(-5);
    const trends = this.calculateTrends(recentData);
    
    this.monitor.emit('trendsAnalyzed', trends);
  }

  private calculateTrends(data: PerformanceReport[]): { [key: string]: 'improving' | 'stable' | 'degrading' } {
    const trends: { [key: string]: 'improving' | 'stable' | 'degrading' } = {};
    
    const metrics = Object.keys(data[0].metrics) as Array<keyof PerformanceMetrics>;
    
    for (const metric of metrics) {
      const values = data.map(report => report.metrics[metric]);
      const trend = this.calculateMetricTrend(values);
      trends[metric] = trend;
    }
    
    return trends;
  }

  private calculateMetricTrend(values: number[]): 'improving' | 'stable' | 'degrading' {
    if (values.length < 2) return 'stable';
    
    const firstHalf = values.slice(0, Math.floor(values.length / 2));
    const secondHalf = values.slice(Math.floor(values.length / 2));
    
    const firstAvg = firstHalf.reduce((a, b) => a + b, 0) / firstHalf.length;
    const secondAvg = secondHalf.reduce((a, b) => a + b, 0) / secondHalf.length;
    
    const change = (secondAvg - firstAvg) / firstAvg;
    
    if (Math.abs(change) < 0.1) return 'stable';
    return change > 0 ? 'degrading' : 'improving';
  }

  public getSessionSummary(): {
    duration: number;
    averageMetrics: PerformanceMetrics;
    totalAlerts: number;
    performanceScore: number;
  } {
    const duration = Date.now() - this.sessionStart;
    
    if (this.data.length === 0) {
      return {
        duration,
        averageMetrics: this.monitor.getMetrics(),
        totalAlerts: 0,
        performanceScore: 100
      };
    }
    
    const averageMetrics = this.calculateAverageMetrics();
    const totalAlerts = this.data.reduce((sum, report) => sum + report.alerts.length, 0);
    const performanceScore = this.calculatePerformanceScore(averageMetrics);
    
    return {
      duration,
      averageMetrics,
      totalAlerts,
      performanceScore
    };
  }

  private calculateAverageMetrics(): PerformanceMetrics {
    if (this.data.length === 0) {
      return this.monitor.getMetrics();
    }
    
    const sum = this.data.reduce((acc, report) => {
      Object.keys(report.metrics).forEach(key => {
        const metric = key as keyof PerformanceMetrics;
        acc[metric] += report.metrics[metric];
      });
      return acc;
    }, {
      compilationTime: 0,
      memoryUsage: 0,
      cpuUsage: 0,
      networkLatency: 0,
      renderTime: 0,
      userInteractionTime: 0
    });
    
    const count = this.data.length;
    return {
      compilationTime: sum.compilationTime / count,
      memoryUsage: sum.memoryUsage / count,
      cpuUsage: sum.cpuUsage / count,
      networkLatency: sum.networkLatency / count,
      renderTime: sum.renderTime / count,
      userInteractionTime: sum.userInteractionTime / count
    };
  }

  private calculatePerformanceScore(metrics: PerformanceMetrics): number {
    const thresholds = this.monitor.getThresholds();
    
    const scores = {
      compilationTime: Math.max(0, 100 - (metrics.compilationTime / thresholds.maxCompilationTime) * 100),
      memoryUsage: Math.max(0, 100 - (metrics.memoryUsage / thresholds.maxMemoryUsage) * 100),
      cpuUsage: Math.max(0, 100 - (metrics.cpuUsage / thresholds.maxCpuUsage) * 100),
      networkLatency: Math.max(0, 100 - (metrics.networkLatency / thresholds.maxNetworkLatency) * 100),
      renderTime: Math.max(0, 100 - (metrics.renderTime / thresholds.maxRenderTime) * 100),
      userInteractionTime: Math.max(0, 100 - (metrics.userInteractionTime / thresholds.maxUserInteractionTime) * 100)
    };
    
    const totalScore = Object.values(scores).reduce((a, b) => a + b, 0) / Object.values(scores).length;
    return Math.round(totalScore);
  }

  public exportAnalytics(): string {
    const analytics = {
      sessionSummary: this.getSessionSummary(),
      trends: this.calculateTrends(this.data),
      dataPoints: this.data.length,
      sessionStart: this.sessionStart,
      sessionEnd: Date.now()
    };
    
    return JSON.stringify(analytics, null, 2);
  }
}
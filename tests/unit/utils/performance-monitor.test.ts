import { describe, test, expect, beforeEach, afterEach, jest } from '@jest/globals';
import { PerformanceMonitor, PerformanceDashboard, PerformanceAnalytics } from '../../../src/utils/performance-monitor';

describe('PerformanceMonitor', () => {
  let monitor: PerformanceMonitor;
  let mockContainer: HTMLElement;

  beforeEach(() => {
    // Create mock DOM container
    mockContainer = document.createElement('div');
    document.body.appendChild(mockContainer);
    
    // Create monitor with custom thresholds for testing
    monitor = new PerformanceMonitor({
      maxCompilationTime: 100,
      maxMemoryUsage: 10 * 1024 * 1024, // 10MB
      maxCpuUsage: 0.5, // 50%
      maxNetworkLatency: 100,
      maxRenderTime: 50,
      maxUserInteractionTime: 25
    });
  });

  afterEach(() => {
    monitor.destroy();
    document.body.removeChild(mockContainer);
  });

  describe('Basic Functionality', () => {
    test('should initialize with default metrics', () => {
      const metrics = monitor.getMetrics();
      
      expect(metrics.compilationTime).toBe(0);
      expect(metrics.memoryUsage).toBe(0);
      expect(metrics.cpuUsage).toBe(0);
      expect(metrics.networkLatency).toBe(0);
      expect(metrics.renderTime).toBe(0);
      expect(metrics.userInteractionTime).toBe(0);
    });

    test('should initialize with custom thresholds', () => {
      const thresholds = monitor.getThresholds();
      
      expect(thresholds.maxCompilationTime).toBe(100);
      expect(thresholds.maxMemoryUsage).toBe(10 * 1024 * 1024);
      expect(thresholds.maxCpuUsage).toBe(0.5);
    });

    test('should start and stop monitoring', (done) => {
      monitor.on('monitoringStarted', () => {
        expect(monitor['monitoringInterval']).toBeTruthy();
        monitor.stopMonitoring();
      });

      monitor.on('monitoringStopped', () => {
        expect(monitor['monitoringInterval']).toBeNull();
        done();
      });

      monitor.startMonitoring(100);
    });
  });

  describe('Alert System', () => {
    test('should create warning alert when metric exceeds threshold', (done) => {
      monitor.on('alert', (alert: any) => {
        expect(alert.type).toBe('warning');
        expect(alert.metric).toBe('compilationTime');
        expect(alert.value).toBe(150);
        expect(alert.threshold).toBe(100);
        expect(alert.message).toContain('exceeded threshold');
        done();
      });

      monitor.recordCompilationTime(150);
    });

    test('should create error alert when metric significantly exceeds threshold', (done) => {
      monitor.on('alert', (alert: any) => {
        expect(alert.type).toBe('error');
        expect(alert.metric).toBe('compilationTime');
        expect(alert.value).toBe(160); // 1.6x threshold
        done();
      });

      monitor.recordCompilationTime(160);
    });

    test('should create critical alert when metric greatly exceeds threshold', (done) => {
      monitor.on('criticalAlert', (alert: any) => {
        expect(alert.type).toBe('critical');
        expect(alert.metric).toBe('compilationTime');
        expect(alert.value).toBe(250); // 2.5x threshold
        done();
      });

      monitor.recordCompilationTime(250);
    });

    test('should store alerts and allow retrieval', () => {
      monitor.recordCompilationTime(150);
      monitor.recordRenderTime(75);
      
      const alerts = monitor.getAlerts();
      
      expect(alerts).toHaveLength(2);
      expect(alerts[0].metric).toBe('compilationTime');
      expect(alerts[1].metric).toBe('renderTime');
    });

    test('should clear alerts', () => {
      monitor.recordCompilationTime(150);
      expect(monitor.getAlerts()).toHaveLength(1);
      
      monitor.clearAlerts();
      expect(monitor.getAlerts()).toHaveLength(0);
    });
  });

  describe('Performance Recording', () => {
    test('should record compilation time', () => {
      monitor.recordCompilationTime(75);
      
      const metrics = monitor.getMetrics();
      expect(metrics.compilationTime).toBe(75);
    });

    test('should record user interaction time', () => {
      monitor.recordUserInteraction('click', 30);
      
      const metrics = monitor.getMetrics();
      expect(metrics.userInteractionTime).toBe(30);
    });

    test('should record render time', () => {
      monitor.recordRenderTime(40);
      
      const metrics = monitor.getMetrics();
      expect(metrics.renderTime).toBe(40);
    });
  });

  describe('Baseline Comparison', () => {
    test('should set and compare with baseline', () => {
      // Set initial metrics
      monitor.recordCompilationTime(50);
      monitor.recordMemoryUsage(5 * 1024 * 1024);
      
      // Set baseline
      monitor.setBaseline();
      
      // Change metrics
      monitor.recordCompilationTime(75);
      monitor.recordMemoryUsage(7 * 1024 * 1024);
      
      // Compare with baseline
      const comparisons = monitor.compareWithBaseline();
      
      expect(comparisons.compilationTime).toBe(50); // 50% increase
      expect(comparisons.memoryUsage).toBe(40); // 40% increase
    });

    test('should emit baseline set event', (done) => {
      monitor.on('baselineSet', (baseline: any) => {
        expect(baseline).toBeTruthy();
        expect(baseline.compilationTime).toBe(50);
        done();
      });

      monitor.recordCompilationTime(50);
      monitor.setBaseline();
    });
  });

  describe('Threshold Management', () => {
    test('should update thresholds', () => {
      monitor.updateThresholds({ maxCompilationTime: 200 });
      
      const thresholds = monitor.getThresholds();
      expect(thresholds.maxCompilationTime).toBe(200);
    });

    test('should emit thresholds updated event', (done) => {
      monitor.on('thresholdsUpdated', (thresholds: any) => {
        expect(thresholds.maxCompilationTime).toBe(200);
        done();
      });

      monitor.updateThresholds({ maxCompilationTime: 200 });
    });
  });

  describe('Data Export/Import', () => {
    test('should export data to JSON', () => {
      monitor.recordCompilationTime(75);
      monitor.recordMemoryUsage(8 * 1024 * 1024);
      
      const exportedData = monitor.exportData();
      const parsedData = JSON.parse(exportedData);
      
      expect(parsedData.metrics.compilationTime).toBe(75);
      expect(parsedData.metrics.memoryUsage).toBe(8 * 1024 * 1024);
      expect(parsedData.timestamp).toBeTruthy();
    });

    test('should import data from JSON', () => {
      const testData = {
        metrics: {
          compilationTime: 100,
          memoryUsage: 15 * 1024 * 1024,
          cpuUsage: 0.6,
          networkLatency: 200,
          renderTime: 75,
          userInteractionTime: 30
        },
        thresholds: {
          maxCompilationTime: 150,
          maxMemoryUsage: 20 * 1024 * 1024
        },
        alerts: [],
        timestamp: Date.now()
      };

      monitor.importData(JSON.stringify(testData));
      
      const metrics = monitor.getMetrics();
      expect(metrics.compilationTime).toBe(100);
      expect(metrics.memoryUsage).toBe(15 * 1024 * 1024);
      
      const thresholds = monitor.getThresholds();
      expect(thresholds.maxCompilationTime).toBe(150);
    });

    test('should emit import error for invalid data', (done) => {
      monitor.on('importError', (error: any) => {
        expect(error).toBeTruthy();
        done();
      });

      monitor.importData('invalid json');
    });
  });

  describe('Report Generation', () => {
    test('should generate performance report', () => {
      monitor.recordCompilationTime(75);
      monitor.recordMemoryUsage(8 * 1024 * 1024);
      monitor.recordRenderTime(40);
      
      const report = monitor['generateReport']();
      
      expect(report.timestamp).toBeTruthy();
      expect(report.metrics.compilationTime).toBe(75);
      expect(report.metrics.memoryUsage).toBe(8 * 1024 * 1024);
      expect(report.metrics.renderTime).toBe(40);
      expect(report.recommendations).toContain('Consider optimizing compilation process');
    });

    test('should emit report generated event', (done) => {
      monitor.on('reportGenerated', (report: any) => {
        expect(report).toBeTruthy();
        expect(report.metrics).toBeTruthy();
        done();
      });

      monitor.recordCompilationTime(75);
      monitor['generateReport']();
    });
  });

  describe('Memory Leak Detection', () => {
    test('should detect memory leaks', () => {
      // Simulate memory leak
      monitor.recordMemoryUsage(5 * 1024 * 1024);
      monitor.recordMemoryUsage(8 * 1024 * 1024);
      monitor.recordMemoryUsage(12 * 1024 * 1024);
      monitor.recordMemoryUsage(18 * 1024 * 1024);
      
      const alerts = monitor.getAlerts();
      const memoryAlerts = alerts.filter(alert => alert.metric === 'memoryUsage');
      
      expect(memoryAlerts.length).toBeGreaterThan(0);
      expect(memoryAlerts.some(alert => alert.type === 'warning')).toBe(true);
    });
  });

  describe('Performance Observer Integration', () => {
    test('should handle performance entries', () => {
      // Mock performance entry
      const mockEntry = {
        entryType: 'measure',
        name: 'compilation',
        duration: 120,
        startTime: performance.now()
      };

      monitor['processPerformanceEntry'](mockEntry as any);
      
      const metrics = monitor.getMetrics();
      expect(metrics.compilationTime).toBe(120);
    });

    test('should handle navigation timing entries', () => {
      const mockEntry = {
        entryType: 'navigation',
        responseStart: 150,
        requestStart: 100,
        duration: 50
      };

      monitor['processPerformanceEntry'](mockEntry as any);
      
      const metrics = monitor.getMetrics();
      expect(metrics.networkLatency).toBe(50);
    });
  });
});

describe('PerformanceDashboard', () => {
  let monitor: PerformanceMonitor;
  let dashboard: PerformanceDashboard;
  let container: HTMLElement;

  beforeEach(() => {
    container = document.createElement('div');
    document.body.appendChild(container);
    
    monitor = new PerformanceMonitor();
    dashboard = new PerformanceDashboard(monitor, container);
  });

  afterEach(() => {
    dashboard.destroy();
    monitor.destroy();
    document.body.removeChild(container);
  });

  test('should create dashboard UI', () => {
    expect(container.querySelector('.performance-dashboard')).toBeTruthy();
    expect(container.querySelector('.metrics-grid')).toBeTruthy();
    expect(container.querySelector('#start-monitoring')).toBeTruthy();
    expect(container.querySelector('#stop-monitoring')).toBeTruthy();
    expect(container.querySelector('#export-data')).toBeTruthy();
    expect(container.querySelector('#clear-alerts')).toBeTruthy();
  });

  test('should update metric cards when metrics are collected', () => {
    monitor.recordCompilationTime(75);
    monitor.recordMemoryUsage(5 * 1024 * 1024);
    
    const compilationValue = container.querySelector('[data-metric="compilationTime"] .metric-value')?.textContent;
    const memoryValue = container.querySelector('[data-metric="memoryUsage"] .metric-value')?.textContent;
    
    expect(compilationValue).toBe('75ms');
    expect(memoryValue).toBe('5.00MB');
  });

  test('should add alerts when performance issues occur', () => {
    monitor.recordCompilationTime(2000); // This should trigger an alert
    
    const alertsContainer = container.querySelector('#alerts-container');
    const alerts = alertsContainer?.querySelectorAll('.alert');
    
    expect(alerts?.length).toBeGreaterThan(0);
    expect(alertsContainer?.textContent).toContain('CRITICAL');
  });

  test('should update recommendations based on performance issues', () => {
    monitor.recordCompilationTime(2000);
    monitor.recordMemoryUsage(50 * 1024 * 1024);
    
    const recommendationsContainer = container.querySelector('#recommendations-container');
    
    expect(recommendationsContainer?.textContent).toContain('Consider optimizing compilation process');
    expect(recommendationsContainer?.textContent).toContain('Monitor memory usage');
  });

  test('should handle control button clicks', () => {
    const startButton = container.querySelector('#start-monitoring') as HTMLButtonElement;
    const stopButton = container.querySelector('#stop-monitoring') as HTMLButtonElement;
    
    // Test start monitoring
    startButton.click();
    expect(monitor['monitoringInterval']).toBeTruthy();
    
    // Test stop monitoring
    stopButton.click();
    expect(monitor['monitoringInterval']).toBeNull();
  });
});

describe('PerformanceAnalytics', () => {
  let monitor: PerformanceMonitor;
  let analytics: PerformanceAnalytics;

  beforeEach(() => {
    monitor = new PerformanceMonitor();
    analytics = new PerformanceAnalytics(monitor);
  });

  afterEach(() => {
    monitor.destroy();
  });

  test('should calculate session summary', () => {
    monitor.recordCompilationTime(75);
    monitor.recordMemoryUsage(8 * 1024 * 1024);
    monitor.recordRenderTime(40);
    
    const summary = analytics.getSessionSummary();
    
    expect(summary.averageMetrics.compilationTime).toBe(75);
    expect(summary.averageMetrics.memoryUsage).toBe(8 * 1024 * 1024);
    expect(summary.averageMetrics.renderTime).toBe(40);
    expect(summary.totalAlerts).toBe(0);
    expect(summary.performanceScore).toBeGreaterThan(0);
    expect(summary.performanceScore).toBeLessThanOrEqual(100);
  });

  test('should calculate performance score', () => {
    monitor.recordCompilationTime(50);
    monitor.recordMemoryUsage(5 * 1024 * 1024);
    monitor.recordCpuUsage(0.3);
    monitor.recordNetworkLatency(100);
    monitor.recordRenderTime(30);
    monitor.recordUserInteractionTime(20);
    
    const summary = analytics.getSessionSummary();
    
    // Should have high performance score for good metrics
    expect(summary.performanceScore).toBeGreaterThan(80);
  });

  test('should export analytics data', () => {
    monitor.recordCompilationTime(75);
    
    const exportedData = analytics.exportAnalytics();
    const parsedData = JSON.parse(exportedData);
    
    expect(parsedData.sessionSummary).toBeTruthy();
    expect(parsedData.trends).toBeTruthy();
    expect(parsedData.dataPoints).toBe(1);
    expect(parsedData.sessionStart).toBeTruthy();
    expect(parsedData.sessionEnd).toBeTruthy();
  });

  test('should analyze trends', () => {
    // Simulate improving trends
    for (let i = 0; i < 5; i++) {
      monitor.recordCompilationTime(100 - i * 10); // Decreasing compilation time
      monitor['generateReport']();
    }

    const mockReport = {
      timestamp: Date.now(),
      metrics: {
        compilationTime: 100,
        memoryUsage: 0,
        cpuUsage: 0,
        networkLatency: 0,
        renderTime: 0,
        userInteractionTime: 0
      },
      thresholds: monitor.getThresholds(),
      alerts: [],
      recommendations: []
    };

    const trends = analytics['calculateTrends']([mockReport]);

    expect(trends.compilationTime).toBe('improving');
  });

  test('should detect degrading trends', () => {
    // Simulate degrading trends
    for (let i = 0; i < 5; i++) {
      monitor.recordCompilationTime(50 + i * 10); // Increasing compilation time
      monitor['generateReport']();
    }

    const mockReport = {
      timestamp: Date.now(),
      metrics: {
        compilationTime: 50,
        memoryUsage: 0,
        cpuUsage: 0,
        networkLatency: 0,
        renderTime: 0,
        userInteractionTime: 0
      },
      thresholds: monitor.getThresholds(),
      alerts: [],
      recommendations: []
    };

    const trends = analytics['calculateTrends']([mockReport]);

    expect(trends.compilationTime).toBe('degrading');
  });

  test('should detect stable trends', () => {
    // Simulate stable trends
    for (let i = 0; i < 5; i++) {
      monitor.recordCompilationTime(75 + (i % 2) * 5); // Small variations around 75
      monitor['generateReport']();
    }

    const mockReport = {
      timestamp: Date.now(),
      metrics: {
        compilationTime: 75,
        memoryUsage: 0,
        cpuUsage: 0,
        networkLatency: 0,
        renderTime: 0,
        userInteractionTime: 0
      },
      thresholds: monitor.getThresholds(),
      alerts: [],
      recommendations: []
    };

    const trends = analytics['calculateTrends']([mockReport]);

    expect(trends.compilationTime).toBe('stable');
  });
});
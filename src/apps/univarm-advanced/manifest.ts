/**
 * Univarm Advanced App Manifest
 * 
 * Production-ready path planning with real Rust backend
 */

export const manifest = {
  id: 'univarm-advanced',
  name: 'Univarm Advanced',
  version: '1.0.0',
  description: 'Production path planning with Rust backend & SSE streaming',
  icon: '🦀',
  category: 'robotics',
  author: 'NAVΛ Studio',
  keywords: ['univarm', 'rust', 'sse', 'ros2', 'path-planning', 'production'],
  
  // Backend requirements
  backend: {
    required: true,
    type: 'rust',
    port: 8080,
    endpoints: [
      '/api/solve',
      '/api/drive/path',
      '/api/drive/sse',
      '/api/ui/presets',
      '/api/actions/catalog',
      '/api/trust/summary',
    ],
  },
  
  // App capabilities
  capabilities: [
    'real-time-solving',
    'sse-streaming',
    'drive-simulation',
    'wheel-telemetry',
    'constraint-monitoring',
    'ros2-integration',
    'grpc-planner',
  ],
  
  // Routes
  routes: [
    {
      path: '/univarm-advanced',
      component: 'UnivarmAdvancedApp',
    },
  ],
  
  // Command palette actions
  actions: [
    {
      id: 'univarm.advanced.solve',
      title: 'Solve path (Advanced)',
      prefix: 'λadv',
      description: 'Calculate optimal path with Rust backend',
      shortcut: 'Cmd+Shift+A',
    },
    {
      id: 'univarm.advanced.drive',
      title: 'Execute drive simulation',
      prefix: 'λdrive',
      description: 'Start wheel simulation with SSE',
    },
  ],
  
  // Dock integration
  dock: {
    enabled: true,
    icon: '🦀',
    name: 'Univarm Pro',
    description: 'Production Path Planning',
  },
};

export default manifest;


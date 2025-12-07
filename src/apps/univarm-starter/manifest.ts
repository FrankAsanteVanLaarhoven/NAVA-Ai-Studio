/**
 * Univarm Starter App Manifest
 * 
 * Defines the app metadata for NAVΛ Studio IDE integration
 */

export const manifest = {
  id: 'univarm-starter',
  name: 'Univarm Starter',
  version: '0.1.0',
  description: 'Path optimization & cross-language code generation',
  icon: '🔷',
  category: 'robotics',
  author: 'NAVΛ Studio',
  keywords: ['univarm', 'path-planning', 'codegen', 'navigation', 'robotics'],
  
  // App capabilities
  capabilities: [
    'path-optimization',
    'code-generation',
    'multi-language-export',
  ],
  
  // Supported export formats
  exportFormats: ['rust', 'cpp', 'python', 'typescript'],
  
  // Routes (if using router)
  routes: [
    {
      path: '/univarm-starter',
      component: 'UnivarmStarterApp',
    },
  ],
  
  // Command palette actions
  actions: [
    {
      id: 'univarm.findOptimalPath',
      title: 'Find optimal path (⋋)',
      prefix: 'λopt',
      description: 'Calculate optimal navigation path',
      shortcut: 'Cmd+Shift+P',
    },
  ],
  
  // Dock/toolbar integration
  dock: {
    enabled: true,
    icon: '🔷',
    name: 'Univarm',
    description: 'Univarm Path Optimizer',
  },
};

export default manifest;


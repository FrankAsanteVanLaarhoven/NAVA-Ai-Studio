const sidebars = {
  docsSidebar: [
    {
      type: 'category',
      label: 'Getting Started',
      collapsed: false,
      items: [
        'intro',
        'installation',
        'quick-start',
        'first-project',
      ],
    },
    {
      type: 'category',
      label: 'Core Concepts',
      collapsed: false,
      items: [
        'concepts/vnc-overview',
        'concepts/navigation-calculus',
        'concepts/lambda-notation',
        'concepts/energy-landscapes',
        'concepts/multi-target-compilation',
      ],
    },
    {
      type: 'category',
      label: 'Features',
      collapsed: false,
      items: [
        'features/editor',
        'features/3d-visualization',
        'features/ros-integration',
        'features/cloud-deployment',
        'features/debugger',
        'features/testing',
      ],
    },
    {
      type: 'category',
      label: 'ROS Learning Center',
      collapsed: true,
      items: [
        'ros/index',
        'ros/basics',
        'ros/navigation',
        'ros/simulation',
        'ros/terminal-commands',
        'ros/examples',
      ],
    },
    {
      type: 'category',
      label: 'Architecture',
      collapsed: true,
      items: [
        'architecture/overview',
        'architecture/frontend',
        'architecture/backend',
        'architecture/lsp',
        'architecture/compiler',
        'architecture/plugin-system',
      ],
    },
    {
      type: 'category',
      label: 'Development',
      collapsed: true,
      items: [
        'development/setup',
        'development/contributing',
        'development/building',
        'development/testing',
        'development/deployment',
      ],
    },
    {
      type: 'category',
      label: 'Advanced',
      collapsed: true,
      items: [
        'advanced/custom-compilers',
        'advanced/plugin-development',
        'advanced/vnc-extensions',
        'advanced/performance-tuning',
        'advanced/troubleshooting',
      ],
    },
  ],

  apiSidebar: [
    {
      type: 'category',
      label: 'API Reference',
      collapsed: false,
      items: [
        'api/index',
        {
          type: 'category',
          label: 'Core Components',
          items: [
            'api/components/editor',
            'api/components/terminal',
            'api/components/visualizer',
            'api/components/compiler',
            'api/components/lsp',
          ],
        },
        {
          type: 'category',
          label: 'ROS Integration',
          items: [
            'api/ros/terminal',
            'api/ros/commands',
            'api/ros/simulation',
            'api/ros/navigation',
          ],
        },
        {
          type: 'category',
          label: 'Utilities',
          items: [
            'api/utils/parser',
            'api/utils/validator',
            'api/utils/formatter',
            'api/utils/config',
          ],
        },
      ],
    },
  ],

  tutorialsSidebar: [
    {
      type: 'category',
      label: 'Tutorials',
      collapsed: false,
      items: [
        'tutorials/index',
        {
          type: 'category',
          label: 'Beginner',
          items: [
            'tutorials/beginner/first-vnc-program',
            'tutorials/beginner/navigation-basics',
            'tutorials/beginner/energy-landscapes',
            'tutorials/beginner/3d-visualization',
          ],
        },
        {
          type: 'category',
          label: 'Intermediate',
          items: [
            'tutorials/intermediate/multi-target-compilation',
            'tutorials/intermediate/ros-integration',
            'tutorials/intermediate/debugging-techniques',
            'tutorials/intermediate/performance-optimization',
          ],
        },
        {
          type: 'category',
          label: 'Advanced',
          items: [
            'tutorials/advanced/custom-compilers',
            'tutorials/advanced/plugin-development',
            'tutorials/advanced/cloud-deployment',
            'tutorials/advanced/vnc-extensions',
          ],
        },
        {
          type: 'category',
          label: 'Video Walkthroughs',
          items: [
            'tutorials/videos/overview',
            'tutorials/videos/installation',
            'tutorials/videos/ros-setup',
            'tutorials/videos/3d-features',
            'tutorials/videos/cloud-deployment',
          ],
        },
      ],
    },
  ],
};

module.exports = sidebars;
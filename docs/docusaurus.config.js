module.exports = {
  title: 'NAVΛ Studio IDE',
  tagline: 'The World\'s First IDE for Van Laarhoven Navigation Calculus Programming',
  url: 'https://docs.navlambda.studio',
  baseUrl: '/',
  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',
  favicon: 'img/favicon.ico',
  organizationName: 'navlambda',
  projectName: 'studio',

  presets: [
    [
      '@docusaurus/preset-classic',
      {
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          editUrl: 'https://github.com/navlambda/studio/tree/main/docs/',
          showLastUpdateAuthor: true,
          showLastUpdateTime: true,
        },
        blog: {
          showReadingTime: true,
          editUrl: 'https://github.com/navlambda/studio/tree/main/docs/',
        },
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      },
    ],
  ],

  themeConfig: {
    navbar: {
      title: 'NAVΛ Studio',
      logo: {
        alt: 'NAVΛ Studio Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'doc',
          docId: 'intro',
          position: 'left',
          label: 'Documentation',
        },
        {
          type: 'doc',
          docId: 'api/index',
          position: 'left',
          label: 'API Reference',
        },
        {
          type: 'doc',
          docId: 'tutorials/index',
          position: 'left',
          label: 'Tutorials',
        },
        {
          to: '/blog',
          label: 'Blog',
          position: 'left'
        },
        {
          href: 'https://github.com/navlambda/studio',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Docs',
          items: [
            {
              label: 'Getting Started',
              to: '/docs/intro',
            },
            {
              label: 'API Reference',
              to: '/docs/api',
            },
            {
              label: 'Tutorials',
              to: '/docs/tutorials',
            },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'Stack Overflow',
              href: 'https://stackoverflow.com/questions/tagged/navlambda',
            },
            {
              label: 'Discord',
              href: 'https://discord.gg/navlambda',
            },
            {
              label: 'Twitter',
              href: 'https://twitter.com/navlambda',
            },
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'Blog',
              to: '/blog',
            },
            {
              label: 'GitHub',
              href: 'https://github.com/navlambda/studio',
            },
            {
              label: 'NPM',
              href: 'https://www.npmjs.com/package/@navlambda/studio',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} NAVΛ Studio. Built with Docusaurus.`,
    },
    prism: {
      theme: require('prism-react-renderer/themes/github'),
      darkTheme: require('prism-react-renderer/themes/dracula'),
      additionalLanguages: ['rust', 'typescript', 'python', 'cpp'],
    },
    algolia: {
      apiKey: 'your-api-key',
      indexName: 'navlambda_docs',
      contextualSearch: true,
    },
  },
};
// @ts-check
const lightCodeTheme = require('prism-react-renderer/themes/github');
const darkCodeTheme = require('prism-react-renderer/themes/dracula');

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'NAVΛ Documentation',
  tagline: 'Language • Math • Verification • SDK',
  url: 'https://example.com',
  baseUrl: '/',
  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',
  favicon: 'img/favicon.ico',
  organizationName: 'univarm', // <-- set to your org/user
  projectName: 'nava-docs',    // <-- set to your repo name

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          routeBasePath: '/', // docs at site root
          sidebarPath: require.resolve('./sidebars.js'),
          editUrl: undefined,
          showLastUpdateTime: true,
        },
        blog: false,
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  plugins: [
    [
      require.resolve('docusaurus-plugin-search-local'),
      { indexBlog: false, indexDocs: true, hashed: true }
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      navbar: {
        title: 'NAVΛ',
        logo: { alt: 'NAVΛ', src: 'img/logo.svg' },
        items: [
          { type: 'docsVersionDropdown', position: 'right' },
          { href: 'https://github.com/your-org/your-repo', label: 'GitHub', position: 'right' },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Docs',
            items: [{ label: 'Overview', to: '/' }],
          },
          {
            title: 'Community',
            items: [
              { label: 'Issues', href: 'https://github.com/your-org/your-repo/issues' },
            ],
          },
        ],
        copyright: `© ${new Date().getFullYear()} NAVΛ authors. Docs under Apache-2.0.`,
      },
      prism: { theme: lightCodeTheme, darkTheme: darkCodeTheme },
    }),
};

module.exports = config;

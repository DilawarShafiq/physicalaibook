// @ts-check
// Note: type annotations allow type checking and IDEs autocompletion

const lightCodeTheme = require('prism-react-renderer').themes.github;
const darkCodeTheme = require('prism-react-renderer').themes.dracula;

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Autosapien Academy',
  tagline: 'Three research-grade books on Physical AI, EHR certification, and Agentic Healthcare',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://DilawarShafiq.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  baseUrl: '/physicalaibook/',

  // GitHub pages deployment config
  organizationName: 'dilawarshafiq',
  projectName: 'physicalaibook',

  onBrokenLinks: 'warn',
  onBrokenAnchors: 'warn',
  markdown: {
    hooks: {
      onBrokenMarkdownLinks: 'warn',
    },
  },
  trailingSlash: false,

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        // Docs are provided as three dedicated plugin instances below.
        docs: false,
        blog: false,
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  plugins: [
    [
      '@docusaurus/plugin-content-docs',
      /** @type {import('@docusaurus/plugin-content-docs').Options} */
      ({
        id: 'physical-ai',
        path: 'books/physical-ai',
        routeBasePath: 'physical-ai',
        sidebarPath: require.resolve('./sidebarsPhysicalAi.js'),
      }),
    ],
    [
      '@docusaurus/plugin-content-docs',
      /** @type {import('@docusaurus/plugin-content-docs').Options} */
      ({
        id: 'cehrs',
        path: 'books/cehrs',
        routeBasePath: 'cehrs',
        sidebarPath: require.resolve('./sidebarsCehrs.js'),
      }),
    ],
    [
      '@docusaurus/plugin-content-docs',
      /** @type {import('@docusaurus/plugin-content-docs').Options} */
      ({
        id: 'agentic-healthcare',
        path: 'books/agentic-healthcare',
        routeBasePath: 'agentic-healthcare',
        sidebarPath: require.resolve('./sidebarsAgenticHealthcare.js'),
      }),
    ],
  ],

  stylesheets: [
    'https://fonts.googleapis.com/css2?family=Inter:wght@300;400;500;600;700&display=swap',
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      navbar: {
        title: 'Autosapien Academy',
        logo: {
          alt: 'Autosapien Academy Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            to: '/physical-ai',
            label: 'Physical AI',
            position: 'left',
          },
          {
            to: '/cehrs',
            label: 'CEHRS Prep',
            position: 'left',
          },
          {
            to: '/agentic-healthcare',
            label: 'Agentic Healthcare',
            position: 'left',
          },
          {
            href: 'https://github.com/DilawarShafiq/physicalaibook',
            label: 'GitHub',
            position: 'right',
          },
          {
            to: '/signin',
            label: 'Sign In',
            position: 'right',
          },
          {
            to: '/signup',
            label: 'Sign Up',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Books',
            items: [
              { label: 'Physical AI & Humanoid Robotics', to: '/physical-ai' },
              { label: 'CEHRS Certification Prep', to: '/cehrs' },
              { label: 'AI Healthcare Employees', to: '/agentic-healthcare' },
            ],
          },
          {
            title: 'Account',
            items: [
              { label: 'Sign In', to: '/signin' },
              { label: 'Sign Up', to: '/signup' },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/DilawarShafiq/physicalaibook',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Autosapien Academy. Built with Docusaurus.`,
      },
      prism: {
        theme: lightCodeTheme,
        darkTheme: darkCodeTheme,
        additionalLanguages: ['bash', 'python', 'yaml'],
      },
    }),
};

module.exports = config;

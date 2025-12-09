// @ts-check
// Note: type annotations allow type checking and IDEs autocompletion

const lightCodeTheme = require('prism-react-renderer').themes.github;
const darkCodeTheme = require('prism-react-renderer').themes.dracula;

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Interactive Textbook for Learning Physical AI',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://DilawarShafiq.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  baseUrl: '/physicalaibook/',

  // GitHub pages deployment config
  organizationName: 'dilawarshafiq',
  projectName: 'physicalaibook',

  onBrokenLinks: 'warn',  // Temporarily change to warn to see build output
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          routeBasePath: '/',
        },
        blog: false,
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
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
        title: 'Physical AI Textbook',
        logo: {
          alt: 'Physical AI Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'doc',
            docId: 'curriculum/introduction',
            position: 'left',
            label: 'Curriculum',
          },
          {
            type: 'doc',
            docId: 'curriculum/learning-outcomes',
            position: 'left',
            label: 'Learning Outcomes',
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
            title: 'Curriculum',
            items: [
              {
                label: 'Introduction',
                to: '/curriculum/introduction',
              },
              {
                label: 'Module 1: ROS 2',
                to: '/curriculum/module-1',
              },
              {
                label: 'Learning Outcomes',
                to: '/curriculum/learning-outcomes',
              },
            ],
          },
          {
            title: 'Resources',
            items: [
              {
                label: 'Assessments',
                to: '/curriculum/assessments',
              },
              {
                label: 'Schedule',
                to: '/curriculum/schedule',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI Textbook. Built with Docusaurus.`,
      },
      prism: {
        theme: lightCodeTheme,
        darkTheme: darkCodeTheme,
        additionalLanguages: ['bash', 'python', 'yaml'],
      },
    }),
};

module.exports = config;

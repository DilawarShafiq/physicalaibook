/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */

// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  curriculumSidebar: [
    {
      type: 'doc',
      id: 'curriculum/introduction',
      label: 'Introduction',
    },
    {
      type: 'doc',
      id: 'curriculum/schedule',
      label: '13-Week Schedule',
    },
    {
      type: 'doc',
      id: 'curriculum/learning-outcomes',
      label: 'Learning Outcomes',
    },
    {
      type: 'doc',
      id: 'curriculum/assessments',
      label: 'Assessments',
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2 Fundamentals',
      items: [
        'curriculum/module-1/index',
        'curriculum/module-1/ros2-basics',
      ],
    },
  ],
};

module.exports = sidebars;

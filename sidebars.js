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
    {
      type: 'category',
      label: 'Module 2: Digital Twin & Gazebo',
      items: [
        'curriculum/module-2/index',
        'curriculum/module-2/gazebo-intro',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: AI-Robot Brain & NVIDIA Isaac',
      items: [
        'curriculum/module-3/index',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: VLA & LLMs for Robotics',
      items: [
        'curriculum/module-4/index',
      ],
    },
    {
      type: 'category',
      label: 'Hardware Requirements',
      items: [
        'hardware/workstation',
        'hardware/edge-kit',
      ],
    },
    {
      type: 'category',
      label: 'System Architecture',
      items: [
        'architecture/system-overview',
      ],
    },
  ],
};

module.exports = sidebars;

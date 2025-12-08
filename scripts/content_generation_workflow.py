"""
Content Generation Workflow for Physical AI & Humanoid Robotics Textbook
Uses Claude Code agents to generate, review, and organize educational content
"""

import asyncio
import os
from pathlib import Path
from typing import List, Dict
import json

from claude_code_agent import ClaudeCodeAgent, ContentSubagents, ContentSpec

class ContentGenerationWorkflow:
    """Orchestrates content generation using Claude Code agents"""
    
    def __init__(self, base_path: str = "docs"):
        self.base_path = Path(base_path)
        self.agent = ClaudeCodeAgent()
        self.subagents = ContentSubagents(self.agent)
    
    async def generate_module_content(self, module_name: str, topics: List[str], 
                                    target_audience: str = "intermediate") -> Dict:
        """Generate content for an entire module"""
        
        results = {
            "module": module_name,
            "topics": {},
            "status": "completed"
        }
        
        for topic in topics:
            print(f"Generating content for {module_name} - {topic}...")
            
            # Select appropriate subagent based on module
            if "ROS 2" in module_name:
                content = await self.subagents.generate_ros2_content(topic, target_audience)
            elif "Gazebo" in module_name or "Digital Twin" in module_name:
                content = await self.subagents.generate_gazebo_content(topic, target_audience)
            elif "Isaac" in module_name or "AI-Robot" in module_name:
                content = await self.subagents.generate_isaac_content(topic, target_audience)
            elif "VLA" in module_name or "LLMs" in module_name:
                content = await self.subagents.generate_vla_content(topic, target_audience)
            else:
                # General subagent for other topics
                spec = ContentSpec(
                    module=module_name,
                    chapter=f"{topic}",
                    topic=topic,
                    target_audience=target_audience,
                    learning_objectives=[f"Understand {topic} concepts", 
                                        f"Apply {topic} in robotics contexts"],
                    prerequisites=["Basic robotics knowledge"],
                    content_type="text and examples",
                    difficulty_level=3
                )
                content = await self.agent.generate_content(spec)
            
            # Review content
            review, review_status = await self.agent.review_content(content, ContentSpec(
                module=module_name,
                chapter=f"{topic}",
                topic=topic,
                target_audience=target_audience,
                learning_objectives=[f"Understand {topic} concepts"],
                prerequisites=["Basic robotics knowledge"],
                content_type="text and examples",
                difficulty_level=3
            ))
            
            results["topics"][topic] = {
                "content_length": len(content),
                "review": review,
                "review_status": review_status,
                "status": "completed"
            }
            
            # Save content to file
            file_path = self.base_path / module_name.lower().replace(" ", "-") / f"{topic.lower().replace(' ', '-')}.md"
            await self.agent.create_content_file(
                ContentSpec(
                    module=module_name,
                    chapter=f"{topic}",
                    topic=topic,
                    target_audience=target_audience,
                    learning_objectives=[f"Understand {topic} concepts"],
                    prerequisites=["Basic robotics knowledge"],
                    content_type="text and examples",
                    difficulty_level=3
                ),
                content,
                str(file_path)
            )
        
        return results
    
    async def generate_curriculum_outline(self) -> str:
        """Generate a comprehensive curriculum outline"""
        
        curriculum_content = """# Physical AI & Humanoid Robotics Curriculum

This curriculum covers the essential topics for understanding and implementing Physical AI systems using humanoid robotics.

## Module 1: ROS 2 Fundamentals

- Introduction to ROS 2 and its architecture
- Nodes, topics, services, and actions
- Parameters and launch files
- tf2 and coordinate transformations
- Quality of Service (QoS) policies

## Module 2: Digital Twin & Gazebo Simulation

- Gazebo simulation environment
- Robot modeling with URDF/SDF
- Sensor simulation and plugins
- Physics engines and parameters
- Simulation-to-reality gap

## Module 3: AI-Robot Brain & NVIDIA Isaac

- NVIDIA Isaac ecosystem
- Perception models deployment
- Isaac Sim for photorealistic training
- Reinforcement learning for manipulation
- Edge AI with Jetson platform

## Module 4: Vision-Language-Action (VLA) & LLMs

- Vision-Language-Action models
- LLM integration with robotics
- Natural language interfaces
- Open-ended manipulation
- Embodied AI concepts

"""
        
        # Save curriculum outline
        await self.agent.create_content_file(
            ContentSpec(
                module="Curriculum Overview",
                chapter="Curriculum Overview",
                topic="Curriculum Overview",
                target_audience="all",
                learning_objectives=["Understand the complete curriculum structure"],
                prerequisites=[],
                content_type="outline",
                difficulty_level=1
            ),
            curriculum_content,
            str(self.base_path / "curriculum" / "overview.md")
        )
        
        return curriculum_content

async def main():
    """Run the complete content generation workflow"""
    
    print("Starting Physical AI & Humanoid Robotics Content Generation...")
    
    workflow = ContentGenerationWorkflow()
    
    # Generate curriculum outline
    print("Generating curriculum outline...")
    await workflow.generate_curriculum_outline()
    
    # Generate content for Module 1: ROS 2 Fundamentals
    print("\nGenerating Module 1: ROS 2 Fundamentals...")
    module1_results = await workflow.generate_module_content(
        "Module 1: ROS 2 Fundamentals",
        [
            "ROS 2 Architecture",
            "Nodes and Topics", 
            "Services and Actions",
            "Parameters and Launch Files",
            "tf2 and Transformations",
            "Quality of Service"
        ],
        "beginner to intermediate"
    )
    
    print(f"Module 1 completed: {len(module1_results['topics'])} topics generated")
    
    # Generate content for Module 2: Digital Twin & Gazebo
    print("\nGenerating Module 2: Digital Twin & Gazebo Simulation...")
    module2_results = await workflow.generate_module_content(
        "Module 2: Digital Twin & Gazebo Simulation",
        [
            "Gazebo Environment",
            "URDF Robot Modeling",
            "Sensor Simulation",
            "Physics Parameters",
            "Simulation Reality Gap"
        ],
        "intermediate"
    )
    
    print(f"Module 2 completed: {len(module2_results['topics'])} topics generated")
    
    # Generate content for Module 3: AI-Robot Brain & NVIDIA Isaac
    print("\nGenerating Module 3: AI-Robot Brain & NVIDIA Isaac...")
    module3_results = await workflow.generate_module_content(
        "Module 3: AI-Robot Brain & NVIDIA Isaac",
        [
            "Isaac Ecosystem",
            "Perception Models",
            "Isaac Sim",
            "Reinforcement Learning",
            "Jetson Platform"
        ],
        "advanced"
    )
    
    print(f"Module 3 completed: {len(module3_results['topics'])} topics generated")
    
    # Generate content for Module 4: VLA & LLMs
    print("\nGenerating Module 4: VLA & LLMs for Robotics...")
    module4_results = await workflow.generate_module_content(
        "Module 4: VLA & LLMs for Robotics",
        [
            "Vision-Language-Action Models",
            "LLM Integration",
            "Natural Language Interfaces",
            "Open-ended Manipulation",
            "Embodied AI"
        ],
        "advanced"
    )
    
    print(f"Module 4 completed: {len(module4_results['topics'])} topics generated")
    
    print("\nContent generation workflow completed!")
    
    # Generate summary
    summary = {
        "modules_completed": 4,
        "topics_completed": (
            len(module1_results['topics']) + 
            len(module2_results['topics']) + 
            len(module3_results['topics']) + 
            len(module4_results['topics'])
        ),
        "total_content_length": sum([
            sum(topic['content_length'] for topic in result['topics'].values())
            for result in [module1_results, module2_results, module3_results, module4_results]
        ])
    }
    
    print(f"Summary: {summary['modules_completed']} modules, {summary['topics_completed']} topics, {summary['total_content_length']:,} total characters")
    
    # Save summary
    with open("content_generation_summary.json", "w") as f:
        json.dump(summary, f, indent=2)

if __name__ == "__main__":
    asyncio.run(main())
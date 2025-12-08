"""
Claude Code Agent for Physical AI & Humanoid Robotics Textbook
Uses Claude's capabilities for content generation, review, and validation
"""

import os
import json
import asyncio
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass
from pathlib import Path

import openai
from openai import AsyncOpenAI

@dataclass
class ContentSpec:
    """Specification for textbook content"""
    module: str
    chapter: str
    topic: str
    target_audience: str  # beginner, intermediate, advanced
    learning_objectives: List[str]
    prerequisites: List[str]
    content_type: str  # text, code, example, exercise
    difficulty_level: int  # 1-5 scale

class ClaudeCodeAgent:
    """Agent for generating and reviewing textbook content using Claude"""
    
    def __init__(self, api_key: Optional[str] = None):
        """Initialize Claude Code Agent"""
        api_key = api_key or os.getenv("OPENAI_API_KEY")
        if not api_key:
            raise ValueError("OPENAI_API_KEY environment variable is required")
        
        self.client = AsyncOpenAI(api_key=api_key)
        self.model = "gpt-4-turbo"  # Using GPT-4 which is compatible with Claude-style reasoning
        
    async def generate_content(self, spec: ContentSpec, additional_context: str = "") -> str:
        """Generate textbook content based on specification"""
        
        system_prompt = f"""
        You are an expert AI and robotics educator specializing in Physical AI & Humanoid Robotics. 
        Your task is to generate high-quality educational content for the Physical AI & Humanoid Robotics textbook.
        
        Content Guidelines:
        - Write in an educational, approachable tone appropriate for {spec.target_audience} learners
        - Include practical examples and code snippets where applicable
        - Use clear, concise language while maintaining technical accuracy
        - Structure content logically with headings, subheadings, and bullet points
        - Include hands-on elements like exercises, labs, or thought experiments
        - Balance theoretical concepts with practical applications
        """
        
        user_prompt = f"""
        Generate content for: {spec.module} - {spec.chapter} - {spec.topic}
        
        Learning Objectives:
        {chr(10).join(f"- {obj}" for obj in spec.learning_objectives)}
        
        Prerequisites:
        {chr(10).join(f"- {prereq}" for prereq in spec.prerequisites)}
        
        Content Type: {spec.content_type}
        Difficulty Level: {spec.difficulty_level}/5
        
        Additional Context:
        {additional_context}
        
        Generate detailed, comprehensive content that addresses the learning objectives. 
        For code examples, use Python and relevant robotics frameworks like ROS 2, OpenCV, PyTorch, etc.
        Include at least one practical example or exercise at the end.
        
        Format the content using markdown suitable for Docusaurus documentation.
        """
        
        try:
            response = await self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                temperature=0.7,
                max_tokens=2000,
            )
            
            return response.choices[0].message.content
            
        except Exception as e:
            print(f"Error generating content: {e}")
            return f"Error generating content: {e}"
    
    async def review_content(self, content: str, spec: ContentSpec) -> Tuple[str, Dict]:
        """Review generated content for accuracy, completeness, and alignment with spec"""
        
        system_prompt = """
        You are an expert reviewer for the Physical AI & Humanoid Robotics textbook.
        Your task is to critically evaluate content for technical accuracy, educational value, 
        and alignment with learning objectives.
        """
        
        user_prompt = f"""
        Review the following content for the Physical AI & Humanoid Robotics textbook:
        
        Spec: {spec.module} - {spec.chapter} - {spec.topic}
        Target Audience: {spec.target_audience}
        Learning Objectives:
        {chr(10).join(f"- {obj}" for obj in spec.learning_objectives)}
        
        Content to Review:
        {content}
        
        Please provide:
        1. A brief summary of the content's strengths
        2. Identified issues or areas for improvement
        3. Suggestions for enhancement
        4. A recommendation (accept, minor revision needed, major revision needed, reject)
        5. Specific technical corrections if applicable
        6. Assessment of how well it aligns with the learning objectives
        """
        
        try:
            response = await self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                temperature=0.3,
                max_tokens=1500,
            )
            
            return response.choices[0].message.content, {"status": "success"}
            
        except Exception as e:
            print(f"Error reviewing content: {e}")
            return f"Error reviewing content: {e}", {"status": "error", "error": str(e)}
    
    async def generate_exercise(self, topic: str, difficulty: int, prerequisites: List[str]) -> str:
        """Generate practice exercises for the topic"""
        
        system_prompt = """
        You are creating practice exercises for the Physical AI & Humanoid Robotics textbook.
        Exercises should be practical, hands-on, and reinforce the key concepts.
        """
        
        user_prompt = f"""
        Create a practical exercise for: {topic}
        Difficulty Level: {difficulty}/5
        Prerequisites: {', '.join(prerequisites)}
        
        The exercise should:
        - Be solvable with the knowledge from the chapter
        - Include clear instructions and expected outcomes
        - Suggest tools, frameworks, or datasets to use
        - Optionally include hints for more difficult problems
        - Be relevant to robotics and AI applications
        - Take approximately 30-60 minutes for the target audience
        
        Format as markdown with clear sections.
        """
        
        try:
            response = await self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                temperature=0.8,
                max_tokens=1000,
            )
            
            return response.choices[0].message.content
            
        except Exception as e:
            print(f"Error generating exercise: {e}")
            return f"Error generating exercise: {e}"
    
    async def create_content_file(self, spec: ContentSpec, content: str, file_path: str) -> str:
        """Create a properly formatted content file"""
        
        # Add frontmatter and structure
        frontmatter = f"""---
title: "{spec.chapter}: {spec.topic}"
sidebar_label: "{spec.topic}"
description: "Learn about {spec.topic} in Physical AI & Humanoid Robotics"
keywords: [physical-ai, robotics, {spec.topic.lower().replace(' ', '-')}, {spec.module.lower().replace(' ', '-')}, textbook]
---

"""
        
        complete_content = frontmatter + content
        
        # Ensure directory exists
        Path(file_path).parent.mkdir(parents=True, exist_ok=True)
        
        # Write content to file
        with open(file_path, 'w', encoding='utf-8') as f:
            f.write(complete_content)
        
        return f"Content successfully written to {file_path}"

# Subagent functions for specific content types
class ContentSubagents:
    """Collection of specialized content generation subagents"""
    
    def __init__(self, agent: ClaudeCodeAgent):
        self.agent = agent
    
    async def generate_ros2_content(self, topic: str, target_audience: str = "intermediate") -> str:
        """Specialized agent for ROS 2 content"""
        spec = ContentSpec(
            module="ROS 2 Fundamentals",
            chapter=f"ROS 2 {topic}",
            topic=topic,
            target_audience=target_audience,
            learning_objectives=[f"Understand core concepts of ROS 2 {topic}", 
                                f"Implement ROS 2 {topic} in practical applications"],
            prerequisites=["Basic Python programming", "Introduction to ROS 2"],
            content_type="text and code",
            difficulty_level=3
        )
        
        additional_context = """
        Focus on ROS 2 Humble Hawksbill distribution. Include code examples in Python 
        using the rclpy library. Explain the architecture differences from ROS 1.
        """
        
        return await self.agent.generate_content(spec, additional_context)
    
    async def generate_gazebo_content(self, topic: str, target_audience: str = "intermediate") -> str:
        """Specialized agent for Gazebo simulation content"""
        spec = ContentSpec(
            module="Digital Twin & Gazebo Simulation",
            chapter=f"Gazebo {topic}",
            topic=topic,
            target_audience=target_audience,
            learning_objectives=[f"Use Gazebo for {topic} in robotics simulation",
                                f"Create realistic simulation environments for {topic}"],
            prerequisites=["Basic ROS 2 knowledge", "Introduction to Gazebo"],
            content_type="text and practical examples",
            difficulty_level=3
        )
        
        additional_context = """
        Include URDF model examples, world creation using SDF, and integration with ROS 2.
        Show practical examples of sensor simulation and physics parameters.
        """
        
        return await self.agent.generate_content(spec, additional_context)
    
    async def generate_isaac_content(self, topic: str, target_audience: str = "advanced") -> str:
        """Specialized agent for NVIDIA Isaac content"""
        spec = ContentSpec(
            module="AI-Robot Brain & NVIDIA Isaac",
            chapter=f"NVIDIA Isaac {topic}",
            topic=topic,
            target_audience=target_audience,
            learning_objectives=[f"Deploy {topic} models using NVIDIA Isaac",
                                f"Integrate AI models with robot control systems"],
            prerequisites=["Deep learning basics", "ROS 2 integration", "NVIDIA Jetson platform"],
            content_type="text and practical examples",
            difficulty_level=4
        )
        
        additional_context = """
        Focus on Isaac ROS and Isaac Sim. Include examples of perception models,
        manipulation tasks, and real-world deployment scenarios.
        """
        
        return await self.agent.generate_content(spec, additional_context)
    
    async def generate_vla_content(self, topic: str, target_audience: str = "advanced") -> str:
        """Specialized agent for Vision-Language-Action content"""
        spec = ContentSpec(
            module="VLA & LLMs for Robotics",
            chapter=f"VLA {topic}",
            topic=topic,
            target_audience=target_audience,
            learning_objectives=[f"Implement {topic} in vision-language-action systems",
                                f"Connect LLMs with robot action execution"],
            prerequisites=["Deep learning", "Vision processing", "Robot control"],
            content_type="text and practical examples",
            difficulty_level=5
        )
        
        additional_context = """
        Include examples of models like RT-2, VLA-1, or similar architectures. 
        Show how to connect LLMs with robot control interfaces.
        """
        
        return await self.agent.generate_content(spec, additional_context)

# Example usage
async def main():
    """Example usage of the Claude Code Agent"""
    
    # Initialize agent
    agent = ClaudeCodeAgent()
    subagents = ContentSubagents(agent)
    
    # Example: Generate ROS 2 Services content
    print("Generating ROS 2 Services content...")
    content = await subagents.generate_ros2_content("Services", "intermediate")
    print(f"Generated content length: {len(content)} characters")
    
    # Review the content
    print("\nReviewing content...")
    review, status = await agent.review_content(content, ContentSpec(
        module="ROS 2 Fundamentals",
        chapter="ROS 2 Services",
        topic="Services",
        target_audience="intermediate",
        learning_objectives=["Understand ROS 2 Services"],
        prerequisites=["ROS 2 Basics"],
        content_type="text and code",
        difficulty_level=3
    ))
    print(f"Review completed: {status['status']}")
    
    # Generate an exercise
    print("\nGenerating exercise...")
    exercise = await agent.generate_exercise("ROS 2 Services", 3, ["ROS 2 Basics", "Topics"])
    print(f"Exercise generated: {len(exercise)} characters")

if __name__ == "__main__":
    asyncio.run(main())
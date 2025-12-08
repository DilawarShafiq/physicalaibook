"""
Reusable Claude Code Subagents for Physical AI & Humanoid Robotics
Provides specialized AI agents for different aspects of the textbook and learning experience
"""

import asyncio
import json
import logging
from typing import Dict, List, Optional, Any, Callable, Awaitable
from dataclasses import dataclass, asdict
from enum import Enum

from openai import AsyncOpenAI

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class AgentType(Enum):
    """Types of specialized agents"""
    CONTENT_GENERATOR = "content_generator"
    TUTOR = "tutor"
    DEBUGGER = "debugger"
    EXERCISE_CREATOR = "exercise_creator"
    CODE_EXPLAINER = "code_explainer"
    PERSONALIZATION_ADVISOR = "personalization_advisor"

@dataclass
class AgentContext:
    """Context information for agents"""
    user_profile: Optional[Dict[str, Any]] = None
    current_content: Optional[str] = None
    selected_text: Optional[str] = None
    learning_history: Optional[List[Dict]] = None
    current_module: Optional[str] = None
    current_topic: Optional[str] = None

@dataclass
class AgentResponse:
    """Response from an agent"""
    content: str
    sources: List[Dict] = None
    confidence: float = 1.0
    metadata: Dict[str, Any] = None

class ClaudeSubagent:
    """Base class for all Claude Code subagents"""
    
    def __init__(self, client: AsyncOpenAI, model: str = "gpt-4-turbo"):
        self.client = client
        self.model = model
        
    async def execute(self, query: str, context: AgentContext) -> AgentResponse:
        """Execute the agent with given query and context"""
        raise NotImplementedError("Subclasses must implement execute method")

class ContentGeneratorAgent(ClaudeSubagent):
    """Agent specialized in generating educational content"""
    
    async def execute(self, query: str, context: AgentContext) -> AgentResponse:
        """Generate educational content based on query and context"""
        
        system_prompt = f"""
        You are an expert educator in Physical AI & Humanoid Robotics. Generate high-quality 
        educational content that is accurate, engaging, and appropriate for the learner's level.
        
        Current context:
        - User profile: {context.user_profile or 'Not available'}
        - Current module: {context.current_module or 'Not specified'}
        - Current topic: {context.current_topic or 'Not specified'}
        """
        
        user_prompt = f"""
        Generate educational content about: {query}
        
        Requirements:
        1. Make it appropriate for the user's experience level: {context.user_profile.get('experience_level', 'intermediate') if context.user_profile else 'intermediate'}
        2. Include practical examples and code snippets where relevant
        3. Use clear, engaging language
        4. Structure with headings, bullet points, and clear explanations
        5. Focus on Physical AI & Robotics applications
        
        Generate comprehensive content that addresses the query in the context of Physical AI.
        """
        
        try:
            response = await self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                temperature=0.7,
                max_tokens=2000
            )
            
            return AgentResponse(
                content=response.choices[0].message.content,
                metadata={"agent_type": AgentType.CONTENT_GENERATOR.value}
            )
        except Exception as e:
            logger.error(f"Error in ContentGeneratorAgent: {e}")
            return AgentResponse(
                content=f"Error generating content: {str(e)}",
                metadata={"agent_type": AgentType.CONTENT_GENERATOR.value, "error": str(e)}
            )

class TutorAgent(ClaudeSubagent):
    """Agent specialized in tutoring and explaining concepts"""
    
    async def execute(self, query: str, context: AgentContext) -> AgentResponse:
        """Explain concepts and answer questions like a tutor"""
        
        system_prompt = f"""
        You are an expert tutor in Physical AI & Humanoid Robotics. Your role is to explain
        concepts clearly, answer questions, and help students understand complex topics.
        
        Teaching approach:
        - Use analogies and simple explanations for beginners
        - Provide deeper technical details for advanced learners
        - Address misconceptions and common pitfalls
        - Encourage active learning through questions
        - Be patient and encouraging
        """
        
        user_prompt = f"""
        Student question: {query}
        
        Context:
        - Current content: {context.current_content[:500] if context.current_content else 'None'}
        - Selected text: {context.selected_text or 'None'}
        - User profile: {context.user_profile or 'Not available'}
        
        Please provide a helpful explanation or answer to the student's question.
        Adapt your response to their experience level and learning needs.
        """
        
        try:
            response = await self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                temperature=0.5,
                max_tokens=1500
            )
            
            return AgentResponse(
                content=response.choices[0].message.content,
                metadata={"agent_type": AgentType.TUTOR.value}
            )
        except Exception as e:
            logger.error(f"Error in TutorAgent: {e}")
            return AgentResponse(
                content=f"Error providing tutoring: {str(e)}",
                metadata={"agent_type": AgentType.TUTOR.value, "error": str(e)}
            )

class DebuggerAgent(ClaudeSubagent):
    """Agent specialized in debugging code and troubleshooting issues"""
    
    async def execute(self, query: str, context: AgentContext) -> AgentResponse:
        """Help debug code and troubleshoot robotics issues"""
        
        system_prompt = f"""
        You are an expert debugger for ROS 2, robotics, and AI code. Help identify issues,
        suggest fixes, and provide troubleshooting guidance.
        
        Focus on:
        - ROS 2 specific issues (nodes, topics, services, actions)
        - Robotics simulation problems
        - AI/ML model integration issues
        - Hardware abstraction problems
        - Provide clear, step-by-step debugging instructions
        """
        
        user_prompt = f"""
        Debugging request: {query}
        
        Context:
        - Current content: {context.current_content[:500] if context.current_content else 'None'}
        - User profile: {context.user_profile or 'Not available'}
        
        Please help debug this issue. If code is provided, analyze it for common problems
        and suggest fixes. If it's a conceptual issue, explain what might be going wrong.
        """
        
        try:
            response = await self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                temperature=0.3,
                max_tokens=1500
            )
            
            return AgentResponse(
                content=response.choices[0].message.content,
                metadata={"agent_type": AgentType.DEBUGGER.value}
            )
        except Exception as e:
            logger.error(f"Error in DebuggerAgent: {e}")
            return AgentResponse(
                content=f"Error with debugging: {str(e)}",
                metadata={"agent_type": AgentType.DEBUGGER.value, "error": str(e)}
            )

class ExerciseCreatorAgent(ClaudeSubagent):
    """Agent specialized in creating practice exercises"""
    
    async def execute(self, query: str, context: AgentContext) -> AgentResponse:
        """Create practice exercises and challenges"""
        
        system_prompt = f"""
        You are an expert at creating practice exercises for Physical AI & Robotics education.
        Create challenging yet achievable exercises that reinforce learning objectives.
        
        Guidelines:
        - Match difficulty to user's experience level
        - Include practical, hands-on exercises
        - Provide clear objectives and success criteria
        - Suggest tools and resources needed
        - Include extension challenges for advanced learners
        """
        
        user_prompt = f"""
        Create an exercise about: {query}
        
        Context:
        - Current topic: {context.current_topic or 'Not specified'}
        - User profile: {context.user_profile or 'Not available'}
        - Current content: {context.current_content[:300] if context.current_content else 'None'}
        
        Create a comprehensive exercise that reinforces the concepts and provides practical
        experience. Include objectives, steps, expected outcomes, and potential challenges.
        """
        
        try:
            response = await self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                temperature=0.8,
                max_tokens=1200
            )
            
            return AgentResponse(
                content=response.choices[0].message.content,
                metadata={"agent_type": AgentType.EXERCISE_CREATOR.value}
            )
        except Exception as e:
            logger.error(f"Error in ExerciseCreatorAgent: {e}")
            return AgentResponse(
                content=f"Error creating exercise: {str(e)}",
                metadata={"agent_type": AgentType.EXERCISE_CREATOR.value, "error": str(e)}
            )

class CodeExplainerAgent(ClaudeSubagent):
    """Agent specialized in explaining code"""
    
    async def execute(self, query: str, context: AgentContext) -> AgentResponse:
        """Explain code snippets and ROS 2 concepts"""
        
        system_prompt = f"""
        You are an expert at explaining code, particularly ROS 2, robotics, and AI code.
        Break down complex code into understandable parts and explain the concepts.
        
        Approach:
        - Explain the purpose of each code section
        - Clarify ROS 2 specific patterns and conventions
        - Highlight important implementation details
        - Relate code to robotics concepts
        - Suggest improvements or best practices when appropriate
        """
        
        user_prompt = f"""
        Please explain this code or concept: {query}
        
        Context:
        - Current content: {context.current_content[:500] if context.current_content else 'None'}
        - User profile: {context.user_profile or 'Not available'}
        
        Break down the code/functions/concepts and explain how they work in the context
        of robotics and Physical AI.
        """
        
        try:
            response = await self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                temperature=0.4,
                max_tokens=1500
            )
            
            return AgentResponse(
                content=response.choices[0].message.content,
                metadata={"agent_type": AgentType.CODE_EXPLAINER.value}
            )
        except Exception as e:
            logger.error(f"Error in CodeExplainerAgent: {e}")
            return AgentResponse(
                content=f"Error explaining code: {str(e)}",
                metadata={"agent_type": AgentType.CODE_EXPLAINER.value, "error": str(e)}
            )

class PersonalizationAdvisorAgent(ClaudeSubagent):
    """Agent specialized in content personalization"""
    
    async def execute(self, query: str, context: AgentContext) -> AgentResponse:
        """Advise on content personalization based on user profile"""
        
        system_prompt = f"""
        You are an expert advisor for personalized learning in Physical AI & Robotics.
        Analyze the user's background and recommend how to adapt learning content.
        
        Considerations:
        - Software development experience level
        - Hardware/robotics experience level
        - Specific interests and goals
        - Learning history and preferences
        - Suggest appropriate depth and focus areas
        """
        
        user_profile_info = json.dumps(context.user_profile, indent=2) if context.user_profile else "No profile available"
        learning_history_info = json.dumps(context.learning_history, indent=2) if context.learning_history else "No history available"
        
        user_prompt = f"""
        Personalization query: {query}
        
        User profile: {user_profile_info}
        Learning history: {learning_history_info}
        Current content: {context.current_content[:300] if context.current_content else 'None'}
        Current topic: {context.current_topic or 'Not specified'}
        
        Provide personalized recommendations for adapting content, focus areas,
        or learning path adjustments based on the user's background and interests.
        """
        
        try:
            response = await self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                temperature=0.6,
                max_tokens=1200
            )
            
            return AgentResponse(
                content=response.choices[0].message.content,
                metadata={"agent_type": AgentType.PERSONALIZATION_ADVISOR.value}
            )
        except Exception as e:
            logger.error(f"Error in PersonalizationAdvisorAgent: {e}")
            return AgentResponse(
                content=f"Error with personalization: {str(e)}",
                metadata={"agent_type": AgentType.PERSONALIZATION_ADVISOR.value, "error": str(e)}
            )

class AgentOrchestrator:
    """Manages and orchestrates multiple Claude Code subagents"""
    
    def __init__(self, api_key: str, model: str = "gpt-4-turbo"):
        self.client = AsyncOpenAI(api_key=api_key)
        self.model = model
        self.agents: Dict[AgentType, ClaudeSubagent] = {}
        self._initialize_agents()
    
    def _initialize_agents(self):
        """Initialize all available agents"""
        self.agents[AgentType.CONTENT_GENERATOR] = ContentGeneratorAgent(self.client, self.model)
        self.agents[AgentType.TUTOR] = TutorAgent(self.client, self.model)
        self.agents[AgentType.DEBUGGER] = DebuggerAgent(self.client, self.model)
        self.agents[AgentType.EXERCISE_CREATOR] = ExerciseCreatorAgent(self.client, self.model)
        self.agents[AgentType.CODE_EXPLAINER] = CodeExplainerAgent(self.client, self.model)
        self.agents[AgentType.PERSONALIZATION_ADVISOR] = PersonalizationAdvisorAgent(self.client, self.model)
    
    async def route_query(self, query: str, context: AgentContext) -> AgentResponse:
        """Determine the best agent for the query and route it"""
        
        # Simple routing logic - could be enhanced with more sophisticated classification
        query_lower = query.lower()
        
        if any(keyword in query_lower for keyword in ['debug', 'error', 'fix', 'problem', 'issue', 'troubleshoot']):
            agent_type = AgentType.DEBUGGER
        elif any(keyword in query_lower for keyword in ['explain', 'what does', 'how does', 'code', 'function']):
            agent_type = AgentType.CODE_EXPLAINER
        elif any(keyword in query_lower for keyword in ['question', 'help', 'understand', 'tutor', 'learn']):
            agent_type = AgentType.TUTOR
        elif any(keyword in query_lower for keyword in ['exercise', 'practice', 'challenge', 'assignment']):
            agent_type = AgentType.EXERCISE_CREATOR
        elif any(keyword in query_lower for keyword in ['personalize', 'adapt', 'recommend', 'suggestions']):
            agent_type = AgentType.PERSONALIZATION_ADVISOR
        else:
            agent_type = AgentType.CONTENT_GENERATOR  # Default to content generation
        
        logger.info(f"Routing query to {agent_type.value} agent")
        agent = self.agents[agent_type]
        return await agent.execute(query, context)
    
    async def execute_agent(self, agent_type: AgentType, query: str, context: AgentContext) -> AgentResponse:
        """Execute a specific agent"""
        if agent_type not in self.agents:
            raise ValueError(f"Unknown agent type: {agent_type}")
        
        agent = self.agents[agent_type]
        return await agent.execute(query, context)
    
    async def batch_execute(self, queries: List[str], context: AgentContext) -> List[AgentResponse]:
        """Execute multiple queries concurrently"""
        tasks = [
            self.route_query(query, context)
            for query in queries
        ]
        return await asyncio.gather(*tasks, return_exceptions=True)

# Example usage and testing
async def main():
    """Example usage of the Claude Code subagent system"""
    
    # This would typically be configured with a real API key
    # For now, showing how it would be used
    import os
    api_key = os.getenv("OPENAI_API_KEY")
    
    if not api_key:
        print("Warning: OPENAI_API_KEY not set. This is just a demonstration.")
        print("To use the agents, set your OpenAI API key in the environment.")
        return
    
    orchestrator = AgentOrchestrator(api_key)
    
    # Example context
    context = AgentContext(
        user_profile={
            "experience_level": "intermediate",
            "background": "software development",
            "robotics_experience": "beginner"
        },
        current_module="ROS 2 Fundamentals",
        current_topic="Services",
        current_content="ROS 2 services provide synchronous request/response communication..."
    )
    
    # Test different types of queries
    test_queries = [
        "Explain how ROS 2 services work compared to topics",
        "I'm getting an error when trying to create a service client",
        "Create a practice exercise for ROS 2 services",
        "How do I use services in my robot project?"
    ]
    
    print("Testing Claude Code Subagents...")
    
    for i, query in enumerate(test_queries, 1):
        print(f"\nQuery {i}: {query}")
        response = await orchestrator.route_query(query, context)
        print(f"Response: {response.content[:200]}...")
        print(f"Agent used: {response.metadata['agent_type']}")
    
    print("\nSubagent system demonstration complete!")

if __name__ == "__main__":
    asyncio.run(main())
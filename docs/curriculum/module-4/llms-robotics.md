# Large Language Models for Robotics

**Week 11: Natural Language Robot Control**

## Overview

Large Language Models (LLMs) enable robots to understand natural language commands, plan tasks, and interact with humans intuitively. This chapter covers LLM integration with robotic systems, prompt engineering, and safe deployment strategies.

## Learning Objectives

- Integrate LLMs (GPT-4, Claude, Gemini) with ROS 2
- Design effective prompts for robot control
- Parse natural language to robot actions
- Implement safety constraints for LLM outputs
- Handle ambiguous and complex commands
- Build voice-controlled robot interfaces

## LLM Integration with ROS 2

### Basic LLM Interface

```python
# llm_commander.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import openai
import json

class LLMCommander(Node):
    """Natural language command interface for robots"""

    def __init__(self):
        super().__init__('llm_commander')

        # OpenAI API (works with OpenAI, Azure, or compatible endpoints)
        self.client = openai.OpenAI(
            api_key=self.declare_parameter('api_key', '').value
        )

        # Subscribe to voice/text commands
        self.command_sub = self.create_subscription(
            String,
            '/voice/command',
            self.command_callback,
            10
        )

        # Publish parsed goals
        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/navigation/goal',
            10
        )

        # Action result publisher
        self.result_pub = self.create_publisher(
            String,
            '/llm/response',
            10
        )

        # System prompt
        self.system_prompt = """You are an AI assistant controlling a mobile robot.

Available capabilities:
- navigate_to(location): Move to a named location
- pick_object(object_name): Pick up an object
- place_object(location): Place held object at location
- inspect(object_name): Take a close look at an object
- say(message): Speak a message

Available locations: kitchen, living_room, bedroom, office
Available objects: cup, book, phone, remote

When given a command, output a JSON array of actions to execute.

Example:
User: "Get me the cup from the kitchen"
Output: [
  {"action": "navigate_to", "params": {"location": "kitchen"}},
  {"action": "pick_object", "params": {"object_name": "cup"}},
  {"action": "navigate_to", "params": {"location": "current_user_location"}},
  {"action": "place_object", "params": {"location": "near_user"}}
]
"""

        self.conversation_history = []

        self.get_logger().info('LLM Commander initialized')

    def command_callback(self, msg: String):
        """Process natural language command"""
        command = msg.data
        self.get_logger().info(f'Received command: {command}')

        # Get LLM response
        action_sequence = self.parse_command(command)

        if action_sequence:
            # Execute actions
            self.execute_actions(action_sequence)
        else:
            self.get_logger().warn('Failed to parse command')
            self.send_response("Sorry, I didn't understand that command.")

    def parse_command(self, command: str):
        """Use LLM to parse command into actions"""
        try:
            # Add command to history
            self.conversation_history.append({
                "role": "user",
                "content": command
            })

            # Call LLM
            response = self.client.chat.completions.create(
                model="gpt-4",
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    *self.conversation_history
                ],
                temperature=0.3,  # Low temperature for consistency
                max_tokens=500
            )

            # Extract actions
            llm_output = response.choices[0].message.content
            self.conversation_history.append({
                "role": "assistant",
                "content": llm_output
            })

            # Parse JSON
            actions = json.loads(llm_output)

            self.get_logger().info(f'Parsed actions: {actions}')
            return actions

        except Exception as e:
            self.get_logger().error(f'LLM parsing failed: {str(e)}')
            return None

    def execute_actions(self, actions):
        """Execute sequence of actions"""
        for action in actions:
            action_type = action['action']
            params = action.get('params', {})

            self.get_logger().info(f'Executing: {action_type} with {params}')

            if action_type == 'navigate_to':
                self.navigate_to(params['location'])
            elif action_type == 'pick_object':
                self.pick_object(params['object_name'])
            elif action_type == 'place_object':
                self.place_object(params['location'])
            elif action_type == 'say':
                self.send_response(params['message'])
            else:
                self.get_logger().warn(f'Unknown action: {action_type}')

    def navigate_to(self, location: str):
        """Send navigation goal"""
        # Map location names to coordinates
        locations = {
            'kitchen': (5.0, 3.0),
            'living_room': (0.0, 0.0),
            'bedroom': (8.0, -2.0),
            'office': (10.0, 5.0)
        }

        if location in locations:
            x, y = locations[location]

            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = x
            goal.pose.position.y = y
            goal.pose.orientation.w = 1.0

            self.goal_pub.publish(goal)
            self.get_logger().info(f'Navigating to {location}')
        else:
            self.get_logger().warn(f'Unknown location: {location}')

    def pick_object(self, object_name: str):
        """Trigger object picking"""
        self.get_logger().info(f'Picking up {object_name}')
        # Call grasp service
        pass

    def place_object(self, location: str):
        """Trigger object placement"""
        self.get_logger().info(f'Placing object at {location}')
        # Call place service
        pass

    def send_response(self, message: str):
        """Send verbal response"""
        response = String()
        response.data = message
        self.result_pub.publish(response)

        # Also send to TTS
        self.get_logger().info(f'Response: {message}')

def main():
    rclpy.init()
    commander = LLMCommander()
    rclpy.spin(commander)
    commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Prompt Engineering for Robotics

### Effective Robot Prompts

```python
# robot_prompts.py

class RobotPrompts:
    """Collection of effective prompts for robot control"""

    @staticmethod
    def task_planning_prompt(available_actions, context):
        """Prompt for high-level task planning"""
        return f"""You are planning actions for a robot assistant.

Current Context:
- Robot location: {context['robot_location']}
- Nearby objects: {', '.join(context['visible_objects'])}
- Holding: {context['holding'] or 'nothing'}

Available Actions:
{chr(10).join(f'- {action}' for action in available_actions)}

Safety Rules:
1. Always verify object location before attempting to pick
2. Navigate to object location before grasping
3. Check if already holding object before picking new one
4. Announce actions using say() for transparency

Given the command, provide a step-by-step action plan in JSON format.
Consider edge cases and failure modes."""

    @staticmethod
    def semantic_understanding_prompt():
        """Prompt for understanding user intent"""
        return """Analyze the user's command and extract:
1. Primary intent (navigate, manipulate, observe, communicate)
2. Target object or location
3. Any constraints or preferences
4. Urgency level (immediate, when convenient, background task)

Output as structured JSON."""

    @staticmethod
    def ambiguity_resolution_prompt(ambiguous_refs, context):
        """Prompt for resolving ambiguous references"""
        return f"""The user said: "{ambiguous_refs['command']}"

There are multiple interpretations:
{chr(10).join(f'- {interp}' for interp in ambiguous_refs['interpretations'])}

Context:
{json.dumps(context, indent=2)}

Which interpretation is most likely? Explain your reasoning and provide
the clarified action sequence."""

    @staticmethod
    def safety_check_prompt(proposed_actions):
        """Prompt for safety validation"""
        return f"""Proposed action sequence:
{json.dumps(proposed_actions, indent=2)}

Perform safety checks:
1. Could any action harm humans?
2. Could any action damage property?
3. Are there safer alternative sequences?
4. Should human confirmation be requested?

Output safety assessment and recommendations."""
```

## Speech Integration

### Voice Command Pipeline

```python
# voice_interface.py
import rclpy
from rclpy.node import Node
from audio_common_msgs.msg import AudioData
from std_msgs.msg import String
import whisper
import numpy as np

class VoiceInterface(Node):
    """Speech-to-text interface for robot control"""

    def __init__(self):
        super().__init__('voice_interface')

        # Load Whisper model
        self.whisper_model = whisper.load_model("base")

        # Subscribe to audio
        self.audio_sub = self.create_subscription(
            AudioData,
            '/audio/input',
            self.audio_callback,
            10
        )

        # Publish transcribed commands
        self.command_pub = self.create_publisher(
            String,
            '/voice/command',
            10
        )

        # Audio buffer
        self.audio_buffer = []
        self.sample_rate = 16000

        # Wake word detection
        self.wake_word = "hey robot"
        self.listening = False

        self.get_logger().info('Voice interface initialized')

    def audio_callback(self, msg: AudioData):
        """Process incoming audio"""
        # Accumulate audio data
        audio_chunk = np.frombuffer(msg.data, dtype=np.int16)
        self.audio_buffer.append(audio_chunk)

        # Process when buffer is large enough (e.g., 3 seconds)
        if len(self.audio_buffer) * len(audio_chunk) > 3 * self.sample_rate:
            self.process_audio()
            self.audio_buffer = []

    def process_audio(self):
        """Transcribe audio and publish command"""
        # Concatenate buffer
        audio = np.concatenate(self.audio_buffer)

        # Normalize
        audio = audio.astype(np.float32) / 32768.0

        # Transcribe with Whisper
        result = self.whisper_model.transcribe(audio)
        text = result['text'].strip().lower()

        self.get_logger().info(f'Transcribed: {text}')

        # Check for wake word
        if not self.listening:
            if self.wake_word in text:
                self.listening = True
                self.get_logger().info('Wake word detected, listening...')
                # Remove wake word from text
                text = text.replace(self.wake_word, '').strip()

        if self.listening and text:
            # Publish command
            command = String()
            command.data = text
            self.command_pub.publish(command)

            # Stop listening after command (can be changed to continuous)
            self.listening = False

def main():
    rclpy.init()
    interface = VoiceInterface()
    rclpy.spin(interface)
    interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Safety and Validation

### Safety Constraint Checker

```python
# safety_checker.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np

class SafetyChecker(Node):
    """Validate LLM-generated actions for safety"""

    def __init__(self):
        super().__init__('safety_checker')

        # Define safety zones
        self.forbidden_zones = [
            {'center': (10.0, 10.0), 'radius': 2.0, 'name': 'stairs'},
            {'center': (5.0, -3.0), 'radius': 1.5, 'name': 'fragile_items'}
        ]

        # Maximum velocities
        self.max_linear_vel = 1.0  # m/s
        self.max_angular_vel = 1.5  # rad/s

        # Restricted objects
        self.restricted_objects = ['knife', 'medication', 'keys']

    def check_navigation_safety(self, goal_pose: PoseStamped) -> tuple:
        """Check if navigation goal is safe"""
        x = goal_pose.pose.position.x
        y = goal_pose.pose.position.y

        # Check forbidden zones
        for zone in self.forbidden_zones:
            dist = np.sqrt(
                (x - zone['center'][0])**2 +
                (y - zone['center'][1])**2
            )
            if dist < zone['radius']:
                return False, f"Goal is in forbidden zone: {zone['name']}"

        # Check workspace bounds
        if not (-10 < x < 10 and -10 < y < 10):
            return False, "Goal is outside workspace bounds"

        return True, "Navigation goal is safe"

    def check_manipulation_safety(self, object_name: str, action: str) -> tuple:
        """Check if manipulation is safe"""

        # Restricted objects
        if object_name.lower() in self.restricted_objects:
            return False, f"Object '{object_name}' is restricted"

        # Action-specific checks
        if action == 'throw':
            return False, "Throwing objects is not allowed"

        return True, "Manipulation is safe"

    def validate_action_sequence(self, actions: list) -> tuple:
        """Validate complete action sequence"""

        for i, action in enumerate(actions):
            action_type = action['action']
            params = action.get('params', {})

            if action_type == 'navigate_to':
                # Would need actual pose, simplified here
                pass

            elif action_type == 'pick_object':
                safe, reason = self.check_manipulation_safety(
                    params.get('object_name', ''),
                    'pick'
                )
                if not safe:
                    return False, f"Action {i+1}: {reason}"

        return True, "All actions validated"

def main():
    rclpy.init()
    checker = SafetyChecker()
    rclpy.spin(checker)
    checker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Lab 11.1: LLM Integration

### Objective
Integrate GPT-4 with ROS 2 command interface.

### Requirements
1. Setup OpenAI API or compatible LLM
2. Design system prompt for robot control
3. Parse commands to action sequences
4. Test with 20+ diverse commands
5. Measure parsing accuracy

## Lab 11.2: Voice Control

### Objective
Build complete voice-controlled robot.

### Requirements
1. Integrate Whisper for speech-to-text
2. Implement wake word detection
3. Connect to LLM commander
4. Add text-to-speech for responses
5. Test with 3+ users

## Lab 11.3: Safety System

### Objective
Implement safety checks for LLM outputs.

### Requirements
1. Define safety constraints and forbidden zones
2. Validate all LLM-generated actions
3. Request human confirmation for risky actions
4. Log all commands and safety checks
5. Test with adversarial prompts

## Summary

This chapter covered:
- **LLM Integration**: Connecting GPT-4/Claude with ROS 2
- **Prompt Engineering**: Effective prompts for robots
- **Voice Interface**: Speech-to-text with Whisper
- **Safety**: Validating LLM outputs
- **Natural Interaction**: Conversational robot control

LLMs enable intuitive human-robot interaction!

---

**Next**: [Vision-Language-Action Models →](./vla-models)
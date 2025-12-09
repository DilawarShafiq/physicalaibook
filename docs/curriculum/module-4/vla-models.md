# Vision-Language-Action Models

**Week 12: End-to-End Multimodal Robot Control**

## Overview

Vision-Language-Action (VLA) models like RT-2, PaLM-E, and OpenVLA combine visual perception, language understanding, and robot control in a single end-to-end model. This chapter covers VLA architectures, fine-tuning, and deployment.

## Learning Objectives

- Understand VLA model architectures (RT-2, OpenVLA)
- Fine-tune VLA models on custom tasks
- Deploy VLA models to edge devices
- Compare VLA vs. modular pipelines
- Handle open-ended manipulation tasks
- Implement zero-shot generalization

## VLA Architecture

### RT-2: Robotics Transformer 2

RT-2 treats robot control as a visual question answering problem:

```
Input: Image + "pick up the red cup"
Output: Sequence of robot actions
```

**Architecture**:
1. **Vision Encoder**: Pre-trained ViT (Vision Transformer)
2. **Language Encoder**: T5 or similar transformer
3. **Action Decoder**: Transformer decoder for action tokens
4. **Action Tokenization**: Discretize continuous actions

### OpenVLA: Open-Source VLA

```python
# openvla_inference.py
import torch
from transformers import AutoModel, AutoTokenizer
from PIL import Image
import numpy as np

class OpenVLAModel:
    """OpenVLA model for robot control"""

    def __init__(self, model_path="openvla-7b"):
        # Load model and tokenizer
        self.model = AutoModel.from_pretrained(model_path)
        self.tokenizer = AutoTokenizer.from_pretrained(model_path)

        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model.to(self.device)
        self.model.eval()

        # Action vocabulary
        self.action_vocab_size = 256
        self.action_bins = np.linspace(-1, 1, self.action_vocab_size)

    def preprocess_image(self, image: Image.Image):
        """Preprocess image for model"""
        # Resize and normalize
        image = image.resize((224, 224))
        image_array = np.array(image).astype(np.float32) / 255.0

        # Convert to tensor
        image_tensor = torch.from_numpy(image_array).permute(2, 0, 1).unsqueeze(0)
        return image_tensor.to(self.device)

    def predict_action(self, image: Image.Image, instruction: str):
        """Predict robot action from image and instruction"""

        # Preprocess inputs
        image_tensor = self.preprocess_image(image)
        text_tokens = self.tokenizer(instruction, return_tensors='pt').to(self.device)

        # Forward pass
        with torch.no_grad():
            outputs = self.model(
                pixel_values=image_tensor,
                input_ids=text_tokens['input_ids']
            )

        # Decode action tokens
        action_tokens = outputs.action_logits.argmax(dim=-1)
        actions = self.detokenize_actions(action_tokens)

        return actions

    def detokenize_actions(self, action_tokens):
        """Convert action tokens to continuous actions"""
        # Map tokens to continuous values
        actions = []
        for token in action_tokens[0]:
            bin_idx = token.item()
            action_value = self.action_bins[bin_idx]
            actions.append(action_value)

        return np.array(actions)

# Usage
model = OpenVLAModel()

# Capture image from camera
image = Image.open("/path/to/camera_image.jpg")

# Get action
instruction = "pick up the red block"
action = model.predict_action(image, instruction)

print(f"Predicted action: {action}")
# Expected output: [x, y, z, roll, pitch, yaw, gripper]
```

## Fine-Tuning VLA Models

### Data Collection

```python
# collect_demonstrations.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
import pickle
from datetime import datetime

class DemonstrationCollector(Node):
    """Collect demonstrations for VLA fine-tuning"""

    def __init__(self):
        super().__init__('demonstration_collector')

        self.bridge = CvBridge()

        # Storage
        self.demonstrations = []
        self.current_demo = {
            'images': [],
            'actions': [],
            'instruction': ''
        }

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10
        )
        self.instruction_sub = self.create_subscription(
            String, '/task_instruction', self.instruction_callback, 10
        )

        # Recording state
        self.recording = False

        self.get_logger().info('Demonstration collector ready')

    def instruction_callback(self, msg: String):
        """Start new demonstration"""
        if not self.recording:
            self.current_demo = {
                'images': [],
                'actions': [],
                'instruction': msg.data,
                'timestamp': datetime.now().isoformat()
            }
            self.recording = True
            self.get_logger().info(f'Recording: {msg.data}')

    def image_callback(self, msg: Image):
        """Collect images"""
        if self.recording:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            self.current_demo['images'].append(cv_image)

    def joint_callback(self, msg: JointState):
        """Collect actions (joint velocities)"""
        if self.recording:
            action = {
                'positions': list(msg.position),
                'velocities': list(msg.velocity),
                'timestamp': msg.header.stamp
            }
            self.current_demo['actions'].append(action)

    def stop_recording(self):
        """Stop and save demonstration"""
        if self.recording:
            self.demonstrations.append(self.current_demo)
            self.recording = False

            # Save to disk
            filename = f"demo_{len(self.demonstrations)}.pkl"
            with open(filename, 'wb') as f:
                pickle.dump(self.current_demo, f)

            self.get_logger().info(
                f'Saved demonstration with {len(self.current_demo["images"])} frames'
            )

def main():
    rclpy.init()
    collector = DemonstrationCollector()

    print("Commands:")
    print("  s - Start recording")
    print("  e - End recording")
    print("  q - Quit")

    rclpy.spin(collector)
    collector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Fine-Tuning Script

```python
# finetune_vla.py
import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader
from transformers import AutoModel, AutoTokenizer
import pickle
from pathlib import Path

class RobotDataset(Dataset):
    """Dataset for VLA fine-tuning"""

    def __init__(self, demo_dir):
        self.demos = []

        # Load all demonstrations
        for demo_file in Path(demo_dir).glob('*.pkl'):
            with open(demo_file, 'rb') as f:
                demo = pickle.load(f)
                self.demos.append(demo)

        print(f"Loaded {len(self.demos)} demonstrations")

    def __len__(self):
        return sum(len(demo['images']) for demo in self.demos)

    def __getitem__(self, idx):
        # Find corresponding demo and frame
        for demo in self.demos:
            if idx < len(demo['images']):
                image = demo['images'][idx]
                action = demo['actions'][idx]['positions']  # Use positions as actions
                instruction = demo['instruction']

                return {
                    'image': torch.from_numpy(image).permute(2, 0, 1).float() / 255.0,
                    'action': torch.tensor(action, dtype=torch.float32),
                    'instruction': instruction
                }
            idx -= len(demo['images'])

def finetune_vla(model_name, demo_dir, output_dir, epochs=10):
    """Fine-tune VLA model on custom demonstrations"""

    # Load model
    model = AutoModel.from_pretrained(model_name)
    tokenizer = AutoTokenizer.from_pretrained(model_name)

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model.to(device)

    # Dataset and dataloader
    dataset = RobotDataset(demo_dir)
    dataloader = DataLoader(dataset, batch_size=8, shuffle=True)

    # Optimizer
    optimizer = torch.optim.AdamW(model.parameters(), lr=1e-5)

    # Loss function
    criterion = nn.MSELoss()

    # Training loop
    model.train()
    for epoch in range(epochs):
        total_loss = 0

        for batch in dataloader:
            images = batch['image'].to(device)
            actions = batch['action'].to(device)
            instructions = batch['instruction']

            # Tokenize instructions
            text_tokens = tokenizer(
                instructions,
                padding=True,
                return_tensors='pt'
            ).to(device)

            # Forward pass
            outputs = model(
                pixel_values=images,
                input_ids=text_tokens['input_ids']
            )

            # Compute loss
            predicted_actions = outputs.action_logits
            loss = criterion(predicted_actions, actions)

            # Backward pass
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

            total_loss += loss.item()

        avg_loss = total_loss / len(dataloader)
        print(f"Epoch {epoch+1}/{epochs}, Loss: {avg_loss:.4f}")

    # Save fine-tuned model
    model.save_pretrained(output_dir)
    tokenizer.save_pretrained(output_dir)
    print(f"Model saved to {output_dir}")

if __name__ == '__main__':
    finetune_vla(
        model_name="openvla-7b",
        demo_dir="./demonstrations",
        output_dir="./finetuned_vla",
        epochs=20
    )
```

## ROS 2 Integration

### VLA Controller Node

```python
# vla_controller.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
from cv_bridge import CvBridge
import torch
from openvla_model import OpenVLAModel

class VLAController(Node):
    """Control robot using VLA model"""

    def __init__(self):
        super().__init__('vla_controller')

        # Load VLA model
        model_path = self.declare_parameter('model_path', 'openvla-7b').value
        self.vla = OpenVLAModel(model_path)

        self.bridge = CvBridge()

        # Current instruction
        self.current_instruction = None

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        self.instruction_sub = self.create_subscription(
            String, '/vla/instruction', self.instruction_callback, 10
        )

        # Publisher for actions
        self.action_pub = self.create_publisher(
            JointState, '/joint_commands', 10
        )

        # Control rate
        self.rate = 10  # Hz
        self.timer = self.create_timer(1.0 / self.rate, self.control_loop)

        self.latest_image = None

        self.get_logger().info('VLA Controller initialized')

    def instruction_callback(self, msg: String):
        """Receive new instruction"""
        self.current_instruction = msg.data
        self.get_logger().info(f'New instruction: {self.current_instruction}')

    def image_callback(self, msg: Image):
        """Store latest image"""
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

    def control_loop(self):
        """Main control loop"""
        if self.latest_image is None or self.current_instruction is None:
            return

        # Get action from VLA model
        action = self.vla.predict_action(self.latest_image, self.current_instruction)

        # Publish action
        joint_cmd = JointState()
        joint_cmd.header.stamp = self.get_clock().now().to_msg()
        joint_cmd.position = action[:6].tolist()  # First 6 DOF
        joint_cmd.velocity = [0.1] * 6  # Velocity for motion

        self.action_pub.publish(joint_cmd)

def main():
    rclpy.init()
    controller = VLAController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Deployment Optimization

### TensorRT Optimization for VLA

```python
# optimize_vla_tensorrt.py
import torch
import torch_tensorrt

def optimize_vla_for_jetson(model_path, output_path):
    """Optimize VLA model with TensorRT"""

    # Load model
    model = torch.jit.load(model_path)
    model.eval().cuda()

    # Example inputs
    image_input = torch.randn(1, 3, 224, 224).cuda()
    text_input = torch.randint(0, 1000, (1, 32)).cuda()

    # Compile with TensorRT
    trt_model = torch_tensorrt.compile(
        model,
        inputs=[
            torch_tensorrt.Input(
                min_shape=[1, 3, 224, 224],
                opt_shape=[1, 3, 224, 224],
                max_shape=[1, 3, 224, 224],
                dtype=torch.float16
            ),
            torch_tensorrt.Input(
                min_shape=[1, 32],
                opt_shape=[1, 32],
                max_shape=[1, 32],
                dtype=torch.int32
            )
        ],
        enabled_precisions={torch.float16},  # FP16 for Jetson
        workspace_size=1 << 30  # 1GB
    )

    # Save optimized model
    torch.jit.save(trt_model, output_path)
    print(f"Optimized model saved to {output_path}")

    # Benchmark
    import time
    num_iterations = 100

    torch.cuda.synchronize()
    start = time.time()

    for _ in range(num_iterations):
        _ = trt_model(image_input, text_input)

    torch.cuda.synchronize()
    end = time.time()

    latency = (end - start) / num_iterations
    fps = 1.0 / latency

    print(f"Latency: {latency*1000:.2f} ms")
    print(f"Throughput: {fps:.2f} FPS")

if __name__ == '__main__':
    optimize_vla_for_jetson(
        'finetuned_vla/model.pt',
        'finetuned_vla/model_trt.pt'
    )
```

## Lab 12.1: VLA Fine-Tuning

### Objective
Fine-tune OpenVLA on custom manipulation tasks.

### Requirements
1. Collect 50+ demonstrations for 3 tasks
2. Fine-tune VLA model
3. Evaluate on held-out test tasks
4. Compare performance to baseline
5. Measure zero-shot generalization

## Lab 12.2: VLA Deployment

### Objective
Deploy VLA model to Jetson Orin.

### Requirements
1. Optimize model with TensorRT
2. Deploy to Jetson AGX Orin
3. Integrate with ROS 2 control
4. Achieve >5 Hz control rate
5. Demonstrate manipulation tasks

## Lab 12.3: VLA vs Modular Pipeline

### Objective
Compare VLA to traditional perception-action pipeline.

### Requirements
1. Implement both approaches for same task
2. Measure success rate, latency, generalization
3. Test on novel objects and instructions
4. Analyze failure modes
5. Document tradeoffs

## Summary

This chapter covered:
- **VLA Architecture**: RT-2, OpenVLA models
- **Fine-Tuning**: Custom task adaptation
- **Deployment**: TensorRT optimization
- **ROS 2 Integration**: Real-time VLA control
- **Comparison**: VLA vs modular approaches

VLAs enable open-ended robot manipulation!

---

**Next**: [Capstone Project & Presentations →](./capstone)
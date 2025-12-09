# Computer Vision for Robotics

**Week 7: Perception Fundamentals**

## Overview

Computer vision enables robots to perceive and understand their environment through visual sensors. This chapter covers essential perception techniques for robotics: object detection, semantic segmentation, depth estimation, and their integration with ROS 2.

## Learning Objectives

By the end of this chapter, you will:
- Implement object detection using YOLO and other modern architectures
- Perform semantic segmentation for scene understanding
- Estimate depth from RGB-D cameras and stereo vision
- Fuse multiple sensor modalities for robust perception
- Deploy vision models in real-time robotic systems
- Integrate perception pipelines with robot control

## Object Detection

### YOLO (You Only Look Once)

YOLO is a state-of-the-art real-time object detection system:

```python
# yolo_detector.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np

class YOLODetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        # Load YOLO model
        self.model = YOLO('yolov8n.pt')  # Nano model for speed
        self.bridge = CvBridge()

        # Subscribe to camera
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publish detections
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/detections',
            10
        )

        # Publish annotated image
        self.annotated_pub = self.create_publisher(
            Image,
            '/detections/image',
            10
        )

        # Detection parameters
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('max_detections', 100)

        self.get_logger().info('YOLO detector initialized')

    def image_callback(self, msg: Image):
        """Process incoming images"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Run detection
            results = self.model(
                cv_image,
                conf=self.get_parameter('confidence_threshold').value,
                iou=self.get_parameter('iou_threshold').value,
                max_det=self.get_parameter('max_detections').value
            )

            # Process results
            detections = Detection2DArray()
            detections.header = msg.header

            for result in results:
                boxes = result.boxes
                for box in boxes:
                    detection = Detection2D()

                    # Bounding box
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    detection.bbox.center.position.x = (x1 + x2) / 2
                    detection.bbox.center.position.y = (y1 + y2) / 2
                    detection.bbox.size_x = x2 - x1
                    detection.bbox.size_y = y2 - y1

                    # Class and confidence
                    detection.results.append({
                        'id': int(box.cls[0]),
                        'score': float(box.conf[0])
                    })

                    detections.detections.append(detection)

                    # Draw on image
                    cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)),
                                (0, 255, 0), 2)
                    label = f"{self.model.names[int(box.cls[0])]} {box.conf[0]:.2f}"
                    cv2.putText(cv_image, label, (int(x1), int(y1) - 10),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Publish detections
            self.detection_pub.publish(detections)

            # Publish annotated image
            annotated_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            annotated_msg.header = msg.header
            self.annotated_pub.publish(annotated_msg)

            self.get_logger().info(
                f'Detected {len(detections.detections)} objects',
                throttle_duration_sec=1.0
            )

        except Exception as e:
            self.get_logger().error(f'Detection failed: {str(e)}')

def main():
    rclpy.init()
    detector = YOLODetector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3D Object Detection

Combining 2D detections with depth information:

```python
# 3d_object_detector.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection3DArray, Detection3D
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import numpy as np
import message_filters

class Detection3D(Node):
    def __init__(self):
        super().__init__('detection_3d')

        self.bridge = CvBridge()

        # Synchronized subscribers for RGB and Depth
        self.rgb_sub = message_filters.Subscriber(self, Image, '/camera/rgb/image_raw')
        self.depth_sub = message_filters.Subscriber(self, Image, '/camera/depth/image_raw')
        self.cam_info_sub = message_filters.Subscriber(self, CameraInfo, '/camera/rgb/camera_info')

        # Time synchronizer
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub, self.cam_info_sub],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.synchronized_callback)

        # Publisher
        self.detection_3d_pub = self.create_publisher(
            Detection3DArray,
            '/detections_3d',
            10
        )

        self.get_logger().info('3D object detector initialized')

    def deproject_pixel_to_point(self, pixel_x, pixel_y, depth, camera_info):
        """Convert 2D pixel + depth to 3D point"""
        # Extract camera intrinsics
        fx = camera_info.k[0]
        fy = camera_info.k[4]
        cx = camera_info.k[2]
        cy = camera_info.k[5]

        # Deproject
        z = depth
        x = (pixel_x - cx) * z / fx
        y = (pixel_y - cy) * z / fy

        return x, y, z

    def synchronized_callback(self, rgb_msg, depth_msg, cam_info_msg):
        """Process synchronized RGB-D frames"""
        try:
            # Convert images
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')

            # Run 2D detection (using YOLO or similar)
            detections_2d = self.run_2d_detection(rgb_image)

            # Convert to 3D detections
            detections_3d = Detection3DArray()
            detections_3d.header = rgb_msg.header

            for det_2d in detections_2d:
                # Get center of bounding box
                center_x = int(det_2d['bbox']['center_x'])
                center_y = int(det_2d['bbox']['center_y'])

                # Get depth at center (with error handling)
                if 0 <= center_y < depth_image.shape[0] and 0 <= center_x < depth_image.shape[1]:
                    depth = depth_image[center_y, center_x]

                    # Skip invalid depths
                    if np.isnan(depth) or depth <= 0:
                        continue

                    # Deproject to 3D
                    x, y, z = self.deproject_pixel_to_point(
                        center_x, center_y, depth, cam_info_msg
                    )

                    # Create 3D detection
                    detection_3d = Detection3D()
                    detection_3d.bbox.center.position.x = x
                    detection_3d.bbox.center.position.y = y
                    detection_3d.bbox.center.position.z = z

                    # Add to array
                    detections_3d.detections.append(detection_3d)

            # Publish 3D detections
            self.detection_3d_pub.publish(detections_3d)

            self.get_logger().info(
                f'Published {len(detections_3d.detections)} 3D detections',
                throttle_duration_sec=1.0
            )

        except Exception as e:
            self.get_logger().error(f'3D detection failed: {str(e)}')

    def run_2d_detection(self, image):
        """Run 2D object detection (placeholder)"""
        # This would call your YOLO detector or similar
        return []

def main():
    rclpy.init()
    detector = Detection3D()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Semantic Segmentation

### DeepLabV3+ for Scene Understanding

```python
# semantic_segmentation.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import torch
import torchvision.transforms as transforms
from torchvision.models.segmentation import deeplabv3_resnet101
import numpy as np

class SemanticSegmentation(Node):
    def __init__(self):
        super().__init__('semantic_segmentation')

        # Load DeepLabV3+ model
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model = deeplabv3_resnet101(pretrained=True).to(self.device)
        self.model.eval()

        self.bridge = CvBridge()

        # Image preprocessing
        self.preprocess = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize(
                mean=[0.485, 0.456, 0.406],
                std=[0.229, 0.224, 0.225]
            )
        ])

        # Subscribe to camera
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publish segmentation mask
        self.mask_pub = self.create_publisher(
            Image,
            '/segmentation/mask',
            10
        )

        # Publish colored visualization
        self.viz_pub = self.create_publisher(
            Image,
            '/segmentation/visualization',
            10
        )

        # PASCAL VOC colors
        self.colors = self.get_pascal_voc_colors()

        self.get_logger().info(f'Semantic segmentation initialized on {self.device}')

    def get_pascal_voc_colors(self):
        """Get PASCAL VOC color palette"""
        return np.array([
            [0, 0, 0], [128, 0, 0], [0, 128, 0], [128, 128, 0],
            [0, 0, 128], [128, 0, 128], [0, 128, 128], [128, 128, 128],
            [64, 0, 0], [192, 0, 0], [64, 128, 0], [192, 128, 0],
            [64, 0, 128], [192, 0, 128], [64, 128, 128], [192, 128, 128],
            [0, 64, 0], [128, 64, 0], [0, 192, 0], [128, 192, 0],
            [0, 64, 128]
        ], dtype=np.uint8)

    def image_callback(self, msg: Image):
        """Process incoming images"""
        try:
            # Convert to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

            # Preprocess
            input_tensor = self.preprocess(cv_image).unsqueeze(0).to(self.device)

            # Run segmentation
            with torch.no_grad():
                output = self.model(input_tensor)['out'][0]
                mask = output.argmax(0).cpu().numpy().astype(np.uint8)

            # Publish grayscale mask
            mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
            mask_msg.header = msg.header
            self.mask_pub.publish(mask_msg)

            # Create colored visualization
            colored_mask = self.colors[mask]

            # Blend with original image
            alpha = 0.5
            blended = (alpha * cv_image + (1 - alpha) * colored_mask).astype(np.uint8)

            # Publish visualization
            viz_msg = self.bridge.cv2_to_imgmsg(blended, encoding='rgb8')
            viz_msg.header = msg.header
            self.viz_pub.publish(viz_msg)

            self.get_logger().info(
                f'Segmentation complete',
                throttle_duration_sec=1.0
            )

        except Exception as e:
            self.get_logger().error(f'Segmentation failed: {str(e)}')

def main():
    rclpy.init()
    segmentor = SemanticSegmentation()
    rclpy.spin(segmentor)
    segmentor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Depth Estimation

### Stereo Vision

```python
# stereo_depth.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from stereo_msgs.msg import DisparityImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import message_filters

class StereoDepth(Node):
    def __init__(self):
        super().__init__('stereo_depth')

        self.bridge = CvBridge()

        # Stereo matcher
        self.stereo = cv2.StereoBM_create(
            numDisparities=16 * 5,  # Must be divisible by 16
            blockSize=15
        )

        # Synchronized subscribers
        self.left_sub = message_filters.Subscriber(
            self, Image, '/stereo/left/image_raw'
        )
        self.right_sub = message_filters.Subscriber(
            self, Image, '/stereo/right/image_raw'
        )
        self.left_info_sub = message_filters.Subscriber(
            self, CameraInfo, '/stereo/left/camera_info'
        )

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.left_sub, self.right_sub, self.left_info_sub],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.stereo_callback)

        # Publishers
        self.disparity_pub = self.create_publisher(
            DisparityImage,
            '/stereo/disparity',
            10
        )

        self.depth_pub = self.create_publisher(
            Image,
            '/stereo/depth',
            10
        )

        self.get_logger().info('Stereo depth estimation initialized')

    def stereo_callback(self, left_msg, right_msg, left_info_msg):
        """Compute depth from stereo pair"""
        try:
            # Convert to grayscale
            left_gray = self.bridge.imgmsg_to_cv2(left_msg, desired_encoding='mono8')
            right_gray = self.bridge.imgmsg_to_cv2(right_msg, desired_encoding='mono8')

            # Compute disparity
            disparity = self.stereo.compute(left_gray, right_gray)

            # Convert to depth
            # depth = (focal_length * baseline) / disparity
            focal_length = left_info_msg.k[0]  # fx
            baseline = 0.12  # meters (typical stereo baseline)

            # Avoid division by zero
            disparity_float = disparity.astype(np.float32) / 16.0  # Fixed-point to float
            depth = np.zeros_like(disparity_float)
            valid = disparity_float > 0
            depth[valid] = (focal_length * baseline) / disparity_float[valid]

            # Publish disparity
            disp_msg = DisparityImage()
            disp_msg.header = left_msg.header
            disp_msg.image = self.bridge.cv2_to_imgmsg(
                (disparity / 16.0).astype(np.float32),
                encoding='32FC1'
            )
            disp_msg.f = focal_length
            disp_msg.t = baseline
            self.disparity_pub.publish(disp_msg)

            # Publish depth
            depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding='32FC1')
            depth_msg.header = left_msg.header
            self.depth_pub.publish(depth_msg)

        except Exception as e:
            self.get_logger().error(f'Stereo depth failed: {str(e)}')

def main():
    rclpy.init()
    stereo = StereoDepth()
    rclpy.spin(stereo)
    stereo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Sensor Fusion

### Multi-Modal Perception

```python
# sensor_fusion.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge
import numpy as np
import message_filters

class SensorFusion(Node):
    """Fuse camera and LIDAR data for robust perception"""

    def __init__(self):
        super().__init__('sensor_fusion')

        self.bridge = CvBridge()

        # Synchronized subscribers
        self.image_sub = message_filters.Subscriber(
            self, Image, '/camera/image_raw'
        )
        self.lidar_sub = message_filters.Subscriber(
            self, LaserScan, '/scan'
        )

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.lidar_sub],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.fusion_callback)

        # Publisher for fused detections
        self.fused_pub = self.create_publisher(
            PoseArray,
            '/fused_detections',
            10
        )

        self.get_logger().info('Sensor fusion initialized')

    def fusion_callback(self, image_msg, scan_msg):
        """Fuse vision and LIDAR data"""
        try:
            # Process vision detections
            vision_detections = self.process_vision(image_msg)

            # Process LIDAR clusters
            lidar_clusters = self.process_lidar(scan_msg)

            # Associate vision detections with LIDAR clusters
            fused_detections = self.associate_detections(
                vision_detections,
                lidar_clusters
            )

            # Publish fused results
            pose_array = PoseArray()
            pose_array.header = image_msg.header
            pose_array.poses = [det['pose'] for det in fused_detections]
            self.fused_pub.publish(pose_array)

            self.get_logger().info(
                f'Fused {len(fused_detections)} detections',
                throttle_duration_sec=1.0
            )

        except Exception as e:
            self.get_logger().error(f'Sensor fusion failed: {str(e)}')

    def process_vision(self, image_msg):
        """Extract objects from camera"""
        # Run object detection (YOLO, etc.)
        return []

    def process_lidar(self, scan_msg):
        """Extract clusters from LIDAR"""
        # Run clustering on laser scan points
        return []

    def associate_detections(self, vision_dets, lidar_clusters):
        """Associate vision detections with LIDAR clusters"""
        # Implement data association (e.g., nearest neighbor, Kalman filter)
        return []

def main():
    rclpy.init()
    fusion = SensorFusion()
    rclpy.spin(fusion)
    fusion.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Lab 7.1: Real-Time Object Detection

### Objective
Deploy YOLOv8 on robot camera stream.

### Requirements
1. Subscribe to camera topic
2. Run YOLO detection at ≥10 FPS
3. Publish detection results
4. Visualize bounding boxes
5. Measure inference latency

## Lab 7.2: Semantic Segmentation

### Objective
Implement semantic segmentation for navigation.

### Requirements
1. Use DeepLabV3+ or similar model
2. Segment drivable surfaces
3. Publish segmentation mask
4. Integrate with navigation costmap
5. Test in simulation and real robot

## Lab 7.3: 3D Perception

### Objective
Build complete 3D perception pipeline.

### Requirements
1. Fuse RGB-D camera data
2. Detect and localize objects in 3D
3. Publish point cloud with detections
4. Visualize in RViz
5. Achieve less than 100ms latency

## Summary

This chapter covered:
- **Object Detection**: YOLO and real-time detection
- **Semantic Segmentation**: Scene understanding
- **Depth Estimation**: Stereo and RGB-D
- **3D Perception**: Combining 2D + depth
- **Sensor Fusion**: Multi-modal integration

You now have the perception skills needed for intelligent robotics!

---

**Next**: [NVIDIA Isaac Sim Fundamentals →](./isaac-sim)
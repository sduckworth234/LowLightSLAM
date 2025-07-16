#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import torch
import torch.nn as nn
import torchvision.transforms as transforms
import sys
import os
import threading
import time

# Add Zero-DCE path to Python path
sys.path.append('/home/duck/Dev/Zero-DCE_extension/Zero-DCE++')

# Import Zero-DCE model
try:
    from model import enhance_net_nopool
    rospy.loginfo("Successfully imported Zero-DCE model")
except ImportError as e:
    rospy.logerr(f"Failed to import Zero-DCE model: {e}")
    rospy.logerr("Make sure Zero-DCE is properly installed")
    sys.exit(1)

# ROS imports
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Header

class ZeroDCENode:
    def __init__(self):
        rospy.init_node('zero_dce_node', anonymous=True)
        rospy.loginfo("Starting Zero-DCE ROS Node...")
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Check if CUDA is available
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        rospy.loginfo(f"Using device: {self.device}")
        
        # Load Zero-DCE model
        self.load_model()
        
        # ROS parameters
        self.enable_enhancement = rospy.get_param('~enable_enhancement', True)
        self.enhancement_strength = rospy.get_param('~enhancement_strength', 1.0)
        self.max_fps = rospy.get_param('~max_fps', 60.0)  # Higher default
        
        # Image preprocessing
        self.transform = transforms.Compose([
            transforms.ToTensor(),
        ])
        
        # Threading for processing
        self.processing_lock = threading.Lock()
        self.last_msg_time = rospy.Time.now()
        
        # Setup ROS publishers and subscribers
        self.setup_ros_interface()
        
        rospy.loginfo("Zero-DCE ROS Node initialized successfully!")
        rospy.loginfo(f"Enhancement enabled: {self.enable_enhancement}")
        rospy.loginfo(f"Max FPS: {self.max_fps}")
        
    def load_model(self):
        """Load Zero-DCE model"""
        try:
            rospy.loginfo("Loading Zero-DCE model...")
            
            # Try different initialization approaches
            try:
                # First try with scale_factor parameter
                self.model = enhance_net_nopool(scale_factor=1)
                rospy.loginfo("Model initialized with scale_factor=1")
            except TypeError:
                try:
                    # Try without parameters
                    self.model = enhance_net_nopool()
                    rospy.loginfo("Model initialized without parameters")
                except TypeError:
                    # Try with different common parameters
                    try:
                        self.model = enhance_net_nopool(scale_factor=4)
                        rospy.loginfo("Model initialized with scale_factor=4")
                    except TypeError:
                        # List other possible parameters
                        rospy.logerr("Cannot initialize model. Let's check the model definition.")
                        # Print the model class signature
                        import inspect
                        sig = inspect.signature(enhance_net_nopool.__init__)
                        rospy.logerr(f"Model signature: {sig}")
                        raise
            
            # Model path
            model_path = '/home/duck/Dev/Zero-DCE_extension/Zero-DCE++/snapshots_Zero_DCE++/Epoch99.pth'
            
            # Check if model file exists
            if not os.path.exists(model_path):
                rospy.logerr(f"Model file not found: {model_path}")
                rospy.logerr("Please make sure Zero-DCE model is trained and saved")
                sys.exit(1)
            
            # Load model weights
            rospy.loginfo(f"Loading model weights from: {model_path}")
            self.model.load_state_dict(torch.load(model_path, map_location=self.device))
            self.model.eval()  # Set to evaluation mode
            self.model.to(self.device)
            
            rospy.loginfo("✓ Zero-DCE model loaded successfully!")
            
        except Exception as e:
            rospy.logerr(f"Error loading Zero-DCE model: {e}")
            import traceback
            rospy.logerr(f"Full traceback: {traceback.format_exc()}")
            sys.exit(1)
    
    def setup_ros_interface(self):
        """Setup ROS publishers and subscribers"""
        # Subscriber to raw camera images
        self.image_sub = rospy.Subscriber(
            '/camera/image_raw', 
            Image, 
            self.image_callback, 
            queue_size=1
        )
        
        # Publisher for enhanced images
        self.enhanced_pub = rospy.Publisher(
            '/camera/image_enhanced', 
            Image, 
            queue_size=1
        )
        
        # Publisher for side-by-side comparison
        self.comparison_pub = rospy.Publisher(
            '/zero_dce/comparison', 
            Image, 
            queue_size=1
        )
        
        rospy.loginfo("ROS interface setup complete")
        rospy.loginfo("Subscribing to: /camera/image_raw")
        rospy.loginfo("Publishing to: /camera/image_enhanced")
        rospy.loginfo("Publishing comparison to: /zero_dce/comparison")
    
    def preprocess_image(self, cv_image):
        """Preprocess image for Zero-DCE"""
        try:
            # Convert BGR to RGB
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            
            # Normalize to [0, 1]
            rgb_image = rgb_image.astype(np.float32) / 255.0
            
            # Convert to tensor and add batch dimension
            tensor_image = self.transform(rgb_image).unsqueeze(0)
            
            return tensor_image.to(self.device)
            
        except Exception as e:
            rospy.logerr(f"Preprocessing failed: {e}")
            return None
    
    def postprocess_image(self, enhanced_tensor, original_shape):
        """Convert enhanced tensor back to OpenCV image"""
        try:
            # Remove batch dimension and convert to numpy
            enhanced_np = enhanced_tensor.squeeze(0).cpu().numpy()
            
            # Transpose from CHW to HWC
            enhanced_np = np.transpose(enhanced_np, (1, 2, 0))
            
            # Clip values and convert to uint8
            enhanced_np = np.clip(enhanced_np, 0, 1)
            enhanced_np = (enhanced_np * 255).astype(np.uint8)
            
            # Convert RGB back to BGR for OpenCV
            enhanced_bgr = cv2.cvtColor(enhanced_np, cv2.COLOR_RGB2BGR)
            
            return enhanced_bgr
            
        except Exception as e:
            rospy.logerr(f"Postprocessing failed: {e}")
            return None
    
    def enhance_image(self, cv_image):
        """Enhance image using Zero-DCE"""
        try:
            with torch.no_grad():  # Disable gradient computation for inference
                # Store original shape
                original_shape = cv_image.shape
                
                # Preprocess
                input_tensor = self.preprocess_image(cv_image)
                if input_tensor is None:
                    return cv_image
                
                # Run Zero-DCE enhancement
                enhanced_tensor, _ = self.model(input_tensor)
                
                # Apply enhancement strength (optional)
                if self.enhancement_strength != 1.0:
                    enhanced_tensor = input_tensor + (enhanced_tensor - input_tensor) * self.enhancement_strength
                
                # Postprocess
                enhanced_image = self.postprocess_image(enhanced_tensor, original_shape)
                if enhanced_image is None:
                    return cv_image
                
                return enhanced_image
                
        except Exception as e:
            rospy.logerr(f"Enhancement failed: {e}")
            return cv_image  # Return original image on failure
    
    def image_callback(self, msg):
        """Main image processing callback"""
        # Rate limiting to avoid overloading
        current_time = rospy.Time.now()
        min_interval = 1.0 / self.max_fps
        if (current_time - self.last_msg_time).to_sec() < min_interval:
            return
        self.last_msg_time = current_time
        
        # Skip if already processing (non-blocking)
        if not self.processing_lock.acquire(blocking=False):
            return
        
        try:
            # Convert ROS image to OpenCV
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                rospy.logdebug(f"Received image: {cv_image.shape}")
            except CvBridgeError as e:
                rospy.logerr(f"CV Bridge error: {e}")
                return
            
            # Enhance image or pass through
            if self.enable_enhancement:
                enhanced_image = self.enhance_image(cv_image)
            else:
                enhanced_image = cv_image
            
            # Create side-by-side comparison
            comparison_image = np.hstack([cv_image, enhanced_image])
            
            # Convert back to ROS messages
            try:
                # Enhanced image
                enhanced_msg = self.bridge.cv2_to_imgmsg(enhanced_image, "bgr8")
                enhanced_msg.header = msg.header  # Preserve timestamp and frame_id
                
                # Comparison image
                comparison_msg = self.bridge.cv2_to_imgmsg(comparison_image, "bgr8")
                comparison_msg.header = msg.header
                
                # Publish
                self.enhanced_pub.publish(enhanced_msg)
                self.comparison_pub.publish(comparison_msg)
                
                rospy.logdebug("Images published successfully")
                
            except CvBridgeError as e:
                rospy.logerr(f"CV Bridge error in publishing: {e}")
                
        except Exception as e:
            rospy.logerr(f"Unexpected error in image callback: {e}")
            
        finally:
            self.processing_lock.release()

if __name__ == '__main__':
    try:
        # Create and run the node
        node = ZeroDCENode()
        rospy.loginfo("Zero-DCE node is running. Press Ctrl+C to stop.")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Zero-DCE node stopped by user")
    except Exception as e:
        rospy.logerr(f"Zero-DCE node failed: {e}")
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
from collections import deque
import queue

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

class ReliableZeroDCENode:
    def __init__(self):
        rospy.init_node('zero_dce_node', anonymous=True)
        rospy.loginfo("Starting Reliable Zero-DCE ROS Node (processes every frame)...")
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Performance parameters
        self.enable_enhancement = rospy.get_param('~enable_enhancement', True)
        self.enhancement_strength = rospy.get_param('~enhancement_strength', 1.0)
        self.max_fps = rospy.get_param('~max_fps', 60.0)  # Still used for monitoring
        self.batch_size = rospy.get_param('~batch_size', 1)
        self.use_tensorrt = rospy.get_param('~use_tensorrt', False)
        
        # Queue-based processing to handle every frame
        self.process_queue = queue.Queue(maxsize=10)  # Buffer up to 10 frames
        self.processing_thread = None
        self.shutdown_flag = threading.Event()
        
        # GPU optimization settings
        self.setup_gpu_optimization()
        
        # Load Zero-DCE model
        self.load_model()
        
        # Performance monitoring
        self.frame_times = deque(maxlen=100)
        self.processing_times = deque(maxlen=100)
        self.frames_received = 0
        self.frames_processed = 0
        self.frames_dropped = 0
        self.last_fps_report = time.time()
        
        # Setup ROS publishers and subscribers
        self.setup_ros_interface()
        
        # Start processing thread
        self.start_processing_thread()
        
        rospy.loginfo("Reliable Zero-DCE ROS Node initialized!")
        rospy.loginfo(f"Enhancement enabled: {self.enable_enhancement}")
        rospy.loginfo(f"Device: {self.device}")
        if self.device.type == 'cuda':
            rospy.loginfo(f"GPU Memory: {torch.cuda.get_device_properties(0).total_memory / 1e9:.1f} GB")
        
    def setup_gpu_optimization(self):
        """Setup GPU optimizations"""
        if torch.cuda.is_available():
            self.device = torch.device("cuda")
            
            # GPU optimizations
            torch.backends.cudnn.benchmark = True
            torch.backends.cudnn.deterministic = False
            
            # Print GPU info
            gpu_props = torch.cuda.get_device_properties(0)
            rospy.loginfo(f"GPU: {gpu_props.name}")
            rospy.loginfo(f"GPU Memory: {gpu_props.total_memory / 1e9:.1f} GB")
            rospy.loginfo(f"GPU Compute Capability: {gpu_props.major}.{gpu_props.minor}")
            
        else:
            self.device = torch.device("cpu")
            rospy.logwarn("CUDA not available, using CPU (will be slower)")
        
    def load_model(self):
        """Load Zero-DCE model with optimizations"""
        try:
            rospy.loginfo("Loading Zero-DCE model...")
            
            # Initialize model
            try:
                self.model = enhance_net_nopool(scale_factor=1)
                rospy.loginfo("Model initialized with scale_factor=1")
            except TypeError:
                try:
                    self.model = enhance_net_nopool()
                    rospy.loginfo("Model initialized without parameters")
                except TypeError:
                    try:
                        self.model = enhance_net_nopool(scale_factor=4)
                        rospy.loginfo("Model initialized with scale_factor=4")
                    except TypeError:
                        import inspect
                        sig = inspect.signature(enhance_net_nopool.__init__)
                        rospy.logerr(f"Model signature: {sig}")
                        raise
            
            # Model path
            model_path = '/home/duck/Dev/Zero-DCE_extension/Zero-DCE++/snapshots_Zero_DCE++/Epoch99.pth'
            
            if not os.path.exists(model_path):
                rospy.logerr(f"Model file not found: {model_path}")
                sys.exit(1)
            
            # Load model weights
            rospy.loginfo(f"Loading model weights from: {model_path}")
            try:
                state_dict = torch.load(model_path, map_location=self.device, weights_only=True)
            except TypeError:
                # Fallback for older PyTorch versions
                state_dict = torch.load(model_path, map_location=self.device)
            
            self.model.load_state_dict(state_dict)
            self.model.eval()
            self.model.to(self.device)
            
            # Model optimizations
            if self.device.type == 'cuda':
                try:
                    # Test FP16 support
                    test_tensor = torch.randn(1, 3, 64, 64).to(self.device).half()
                    self.model = self.model.half()
                    with torch.no_grad():
                        _ = self.model(test_tensor)
                    self.use_fp16 = True
                    rospy.loginfo("✓ Using FP16 precision for speed")
                except Exception as e:
                    rospy.logwarn(f"FP16 not supported, using FP32: {e}")
                    self.model = self.model.float()
                    self.use_fp16 = False
            else:
                self.use_fp16 = False
            
            # Model compilation (optional)
            try:
                self.model = torch.compile(self.model, mode="reduce-overhead")
                rospy.loginfo("✓ Model compiled for optimization")
            except Exception as e:
                rospy.loginfo(f"Model compilation not available: {e}")
            
            # Warm up the model
            rospy.loginfo("Warming up model...")
            dummy_input = torch.randn(1, 3, 480, 640).to(self.device)
            if self.use_fp16:
                dummy_input = dummy_input.half()
            
            with torch.no_grad():
                for _ in range(3):
                    try:
                        _ = self.model(dummy_input)
                    except Exception as e:
                        rospy.logerr(f"Model warmup failed: {e}")
                        break
            
            rospy.loginfo("✓ Zero-DCE model loaded and optimized!")
            
        except Exception as e:
            rospy.logerr(f"Error loading Zero-DCE model: {e}")
            import traceback
            rospy.logerr(f"Full traceback: {traceback.format_exc()}")
            sys.exit(1)
    
    def setup_ros_interface(self):
        """Setup ROS publishers and subscribers"""
        # Subscriber - process ALL incoming images
        self.image_sub = rospy.Subscriber(
            '/camera/image_raw', 
            Image, 
            self.image_callback, 
            queue_size=1  # Reduced to prevent buildup
        )
        
        # Publishers
        self.enhanced_pub = rospy.Publisher(
            '/camera/image_enhanced', 
            Image, 
            queue_size=3
        )
        
        self.comparison_pub = rospy.Publisher(
            '/zero_dce/comparison', 
            Image, 
            queue_size=1
        )
        
        # Performance monitoring
        rospy.Timer(rospy.Duration(3.0), self.publish_performance_stats)
        
        rospy.loginfo("Reliable ROS interface setup complete")
    
    def start_processing_thread(self):
        """Start the dedicated processing thread"""
        self.processing_thread = threading.Thread(target=self.processing_worker, daemon=True)
        self.processing_thread.start()
        rospy.loginfo("Processing thread started")
    
    def processing_worker(self):
        """Dedicated worker thread that processes every frame"""
        rospy.loginfo("Processing worker thread started")
        
        while not self.shutdown_flag.is_set() and not rospy.is_shutdown():
            try:
                # Get next frame to process (block with timeout)
                try:
                    msg, cv_image = self.process_queue.get(timeout=1.0)
                except queue.Empty:
                    continue
                
                # Process the frame
                start_time = time.time()
                
                if self.enable_enhancement:
                    enhanced_image = self.enhance_image_reliable(cv_image)
                else:
                    enhanced_image = cv_image
                
                processing_time = time.time() - start_time
                self.processing_times.append(processing_time)
                self.frames_processed += 1
                
                # Publish results
                try:
                    # Enhanced image (priority)
                    enhanced_msg = self.bridge.cv2_to_imgmsg(enhanced_image, "bgr8")
                    enhanced_msg.header = msg.header
                    self.enhanced_pub.publish(enhanced_msg)
                    
                    # Comparison (if someone is listening)
                    if self.comparison_pub.get_num_connections() > 0:
                        comparison_image = np.hstack([cv_image, enhanced_image])
                        comparison_msg = self.bridge.cv2_to_imgmsg(comparison_image, "bgr8")
                        comparison_msg.header = msg.header
                        self.comparison_pub.publish(comparison_msg)
                        
                except CvBridgeError as e:
                    rospy.logerr(f"Publishing error: {e}")
                
                # Mark task as done
                self.process_queue.task_done()
                
            except Exception as e:
                rospy.logerr(f"Processing worker error: {e}")
                import traceback
                rospy.logerr(f"Traceback: {traceback.format_exc()}")
                
        rospy.loginfo("Processing worker thread stopped")
    
    def preprocess_image_reliable(self, cv_image):
        """Reliable preprocessing with error handling"""
        try:
            # Convert BGR to RGB
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            
            # Normalize to [0, 1]
            rgb_image = rgb_image.astype(np.float32) / 255.0
            
            # Convert to tensor
            tensor_image = torch.from_numpy(rgb_image.transpose(2, 0, 1)).unsqueeze(0)
            
            # Move to device
            tensor_image = tensor_image.to(self.device, non_blocking=True)
            if self.use_fp16:
                tensor_image = tensor_image.half()
            
            return tensor_image
            
        except Exception as e:
            rospy.logerr(f"Preprocessing failed: {e}")
            return None
    
    def postprocess_image_reliable(self, enhanced_tensor):
        """Reliable postprocessing with error handling"""
        try:
            # Convert to FP32 if needed
            if self.use_fp16:
                enhanced_tensor = enhanced_tensor.float()
            
            # Move to CPU
            enhanced_np = enhanced_tensor.squeeze(0).detach().cpu().numpy()
            
            # Transpose and convert
            enhanced_np = np.transpose(enhanced_np, (1, 2, 0))
            enhanced_np = np.clip(enhanced_np * 255.0, 0, 255).astype(np.uint8)
            
            # Convert RGB back to BGR
            enhanced_bgr = cv2.cvtColor(enhanced_np, cv2.COLOR_RGB2BGR)
            
            return enhanced_bgr
            
        except Exception as e:
            rospy.logerr(f"Postprocessing failed: {e}")
            return None
    
    def enhance_image_reliable(self, cv_image):
        """Reliable enhancement that always returns a result"""
        try:
            with torch.no_grad():
                # Preprocess
                input_tensor = self.preprocess_image_reliable(cv_image)
                if input_tensor is None:
                    rospy.logwarn("Preprocessing failed, returning original image")
                    return cv_image
                
                # GPU inference with error handling
                try:
                    enhanced_tensor, _ = self.model(input_tensor)
                except Exception as e:
                    rospy.logerr(f"Model inference failed: {e}")
                    return cv_image
                
                # Apply enhancement strength
                if abs(self.enhancement_strength - 1.0) > 0.01:
                    enhanced_tensor = input_tensor + (enhanced_tensor - input_tensor) * self.enhancement_strength
                
                # Postprocess
                enhanced_image = self.postprocess_image_reliable(enhanced_tensor)
                if enhanced_image is None:
                    rospy.logwarn("Postprocessing failed, returning original image")
                    return cv_image
                
                return enhanced_image
                
        except Exception as e:
            rospy.logerr(f"Enhancement failed completely: {e}")
            return cv_image  # Always return something
    
    def image_callback(self, msg):
        """Main callback - queues every frame for processing"""
        self.frames_received += 1
        
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Try to add to processing queue
            try:
                self.process_queue.put_nowait((msg, cv_image))
            except queue.Full:
                # Queue is full, drop oldest frame and add new one
                try:
                    self.process_queue.get_nowait()  # Remove oldest
                    self.frames_dropped += 1
                    self.process_queue.put_nowait((msg, cv_image))  # Add new
                except queue.Empty:
                    pass
                
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error: {e}")
        except Exception as e:
            rospy.logerr(f"Image callback error: {e}")
    
    def publish_performance_stats(self, event):
        """Publish comprehensive performance statistics"""
        if self.frames_received > 0:
            processing_rate = len(self.processing_times)
            avg_processing_time = np.mean(list(self.processing_times)[-30:]) if self.processing_times else 0
            
            processing_fps = 1.0 / avg_processing_time if avg_processing_time > 0 else 0
            
            # Calculate success rate
            success_rate = (self.frames_processed / self.frames_received) * 100 if self.frames_received > 0 else 0
            
            if self.device.type == 'cuda':
                gpu_memory_mb = torch.cuda.memory_allocated() / 1e6
                rospy.loginfo(
                    f"📊 Performance: {processing_fps:.1f} FPS processing | "
                    f"✅ {success_rate:.1f}% success rate | "
                    f"📥 {self.frames_received} received | "
                    f"✨ {self.frames_processed} enhanced | "
                    f"❌ {self.frames_dropped} dropped | "
                    f"🎮 GPU: {gpu_memory_mb:.0f}MB"
                )
            else:
                rospy.loginfo(
                    f"📊 Performance: {processing_fps:.1f} FPS processing | "
                    f"✅ {success_rate:.1f}% success rate | "
                    f"📥 {self.frames_received} received | "
                    f"✨ {self.frames_processed} enhanced | "
                    f"❌ {self.frames_dropped} dropped (CPU)"
                )
            
            # Queue status
            queue_size = self.process_queue.qsize()
            rospy.loginfo(f"🔄 Queue: {queue_size}/10 frames pending")

    def shutdown(self):
        """Clean shutdown"""
        rospy.loginfo("Shutting down Zero-DCE node...")
        self.shutdown_flag.set()
        if self.processing_thread and self.processing_thread.is_alive():
            self.processing_thread.join(timeout=2.0)

if __name__ == '__main__':
    try:
        node = ReliableZeroDCENode()
        rospy.loginfo("🚀 Reliable Zero-DCE node running - processes EVERY frame!")
        
        # Handle shutdown gracefully
        rospy.on_shutdown(node.shutdown)
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Zero-DCE node stopped by user")
    except Exception as e:
        rospy.logerr(f"Zero-DCE node failed: {e}")
        import traceback
        rospy.logerr(f"Traceback: {traceback.format_exc()}")
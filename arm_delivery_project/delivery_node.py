import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import random

class DeliveryNode(Node):
    def __init__(self):
        super().__init__('delivery_node')
        
        # Subscriber: listens for delivery requests
        self.subscription = self.create_subscription(
            String,
            'deliver_item',
            self.listener_callback,
            10
        )
        
        # Publisher: sends visualization markers to RViz
        self.marker_pub = self.create_publisher(Marker, '/marker', 10)
        
        # Simulated 3D stack of items
        self.stack = [
            [[f"Item-{x}-{y}-{z}" for z in range(3)] for y in range(3)]
            for x in range(3)
        ]
        
        # Display stack visualization at startup
        self.visualize_stack()
        
        self.get_logger().info("Delivery node has started. Ready to process delivery requests.")
        self.get_logger().info("Send commands like: ros2 topic pub --once /deliver_item std_msgs/String \"data: '1,2,0'\"")
    
    def listener_callback(self, msg):
        try:
            # Parse the message data to get x, y, z values
            coords = msg.data.strip().split(',')
            if len(coords) != 3:
                raise ValueError("Need exactly 3 coordinates: x,y,z")
                
            x, y, z = map(int, coords)
            
            # Validate coordinates
            if not (0 <= x < 3 and 0 <= y < 3 and 0 <= z < 3):
                raise ValueError("Coordinates must be between 0 and 2")
            
            # Clear previous markers
            self.clear_markers()
            
            # Show shuffling animation
            self.get_logger().info(f"Starting delivery process for coordinates ({x},{y},{z})")
            
            # Shuffle items above the target one
            for zi in range(2, z, -1):
                item = self.stack[x][y][zi]
                self.get_logger().info(f"Shuffling: Moving {item} aside")
            
            # Deliver the item
            item_to_deliver = self.stack[x][y][z]
            self.get_logger().info(f"Delivering {item_to_deliver}")
            
            # Publish the marker to visualize in RViz
            self.publish_delivery_marker(x, y, z)
            
        except Exception as e:
            self.get_logger().error(f"Invalid input. Use format: x,y,z -- Error: {e}")
    
    def clear_markers(self):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "delivery_marker"
        marker.action = Marker.DELETEALL
        self.marker_pub.publish(marker)
    
    def visualize_stack(self):
        # Create markers for each position in the stack
        for x in range(3):
            for y in range(3):
                for z in range(3):
                    marker = Marker()
                    marker.header.frame_id = "base_link"
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.ns = "stack_markers"
                    marker.id = x * 100 + y * 10 + z
                    marker.type = Marker.CUBE
                    marker.action = Marker.ADD
                    
                    # Position
                    marker.pose.position.x = float(x) * 0.15
                    marker.pose.position.y = float(y) * 0.15
                    marker.pose.position.z = float(z) * 0.15 + 0.3
                    marker.pose.orientation.w = 1.0
                    
                    # Size
                    marker.scale.x = 0.05
                    marker.scale.y = 0.05
                    marker.scale.z = 0.05
                    
                    # Color - slight variation for each item
                    marker.color.a = 0.5  # Semi-transparent
                    marker.color.r = 0.2 + (x / 10.0)
                    marker.color.g = 0.2 + (y / 10.0)
                    marker.color.b = 0.2 + (z / 10.0)
                    
                    # Make it permanent
                    marker.lifetime.sec = 0
                    
                    self.marker_pub.publish(marker)
    
    def publish_delivery_marker(self, x, y, z):
        # Create a marker to visualize in RViz
        marker = Marker()
        marker.header.frame_id = "base_link"  # Robot base frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "delivery_marker"
        marker.id = 1000  # Different ID than stack markers
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # Set position based on the received x, y, z
        marker.pose.position.x = float(x) * 0.15
        marker.pose.position.y = float(y) * 0.15
        marker.pose.position.z = float(z) * 0.15 + 0.3
        marker.pose.orientation.w = 1.0  # No rotation
        
        # Set the size of the cube (larger than stack items)
        marker.scale.x = 0.08
        marker.scale.y = 0.08
        marker.scale.z = 0.08
        
        # Set color (bright green for visibility)
        marker.color.a = 1.0  # Full opacity
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        
        # Set marker lifetime (15 seconds)
        marker.lifetime.sec = 15
        
        # Add animation points
        self.animate_delivery(x, y, z)
        
        # Publish the marker
        self.marker_pub.publish(marker)
        self.get_logger().info(f"Published delivery marker at position ({x},{y},{z})")
    
    def animate_delivery(self, x, y, z):
        # Create an arrow showing the delivery path
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "delivery_animation"
        marker.id = 2000
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Start at the end effector position (approximately)
        marker.points = []
        
        start_point = Point()
        start_point.x = 0.0
        start_point.y = 0.0
        start_point.z = 0.3
        
        end_point = Point()
        end_point.x = float(x) * 0.15
        end_point.y = float(y) * 0.15
        end_point.z = float(z) * 0.15 + 0.3
        
        marker.points.append(start_point)
        marker.points.append(end_point)
        
        # Set the size of the arrow
        marker.scale.x = 0.02  # shaft diameter
        marker.scale.y = 0.04  # head diameter
        marker.scale.z = 0.01  # head length
        
        # Set color (yellow for the animation)
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        
        # Set marker lifetime
        marker.lifetime.sec = 5
        
        # Publish the marker
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = DeliveryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

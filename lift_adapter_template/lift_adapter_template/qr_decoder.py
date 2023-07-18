#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from pyzbar import pyzbar
from rmf_lift_msgs.msg import LiftFloor


class QRCodeDetectorNode(Node):
	def __init__(self):
		super().__init__('qr_code_detector')
		self.get_logger().info("QR Code Decoder initializing ...")
		self.subscription = self.create_subscription(Image, '/image', self.image_callback, 10)
		self.cv_bridge = CvBridge()
		
	def image_callback(self, msg: Image):
		cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
		gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
		
		qr_codes = pyzbar.decode(gray_image)
		
		for qr_code in qr_codes:
			x, y, w, h = qr_code.rect
			barcode_data = qr_code.data.decode('utf-8')
			barcode_type = qr_code.type
			
			#DEBUGGING: RAW QR CODE DATA
			#self.get_logger().info("Raw data:" + barcode_data)
			
			# Extract numbers from barcode_data string
			block = None
			floor = None
			try:
			# Split the barcode_data string into substrings based on commas and equal sign
				parts = barcode_data.split(', ')
				for part in parts:
					if part.startswith("Block"):
						block = int(part.split('=')[1].strip())
					elif part.startswith("Floor"):
						floor = int(part.split('=')[1].strip())

				if block is not None and floor is not None:
					#self.get_logger().info("Block: "+str(block))
					self.get_logger().info("Floor: "+str(floor))
					
					#Publishes the current lift floor
					self.lift_current_floor_pub_ = self.create_publisher(LiftFloor, "/lift_current_floor", 10)
					self.msg = LiftFloor()
					self.msg.lift_current_floor = floor
					self.lift_current_floor_pub_.publish(self.msg)
				else:
					self.get_logger().warning("Invalid barcode_data format.")
			except (IndexError, ValueError):
                		self.get_logger().warning("Failed to extract numbers from barcode_data.")
			

def main(args=None):
	rclpy.init(args=args)
	qr_code_detector = QRCodeDetectorNode()
	rclpy.spin(qr_code_detector)
	qr_code_detector.destroy_node()
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()

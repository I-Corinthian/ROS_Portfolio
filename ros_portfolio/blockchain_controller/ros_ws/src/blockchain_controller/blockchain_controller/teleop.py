#!/use/bin/env python3
import rclpy
import os
from rclpy.node import Node
from geometry_msgs.msg import Twist
from substrateinterface import SubstrateInterface, Keypair
from substrateinterface.contracts import ContractInstance, ContractCode

class ChainTeleop(Node):

    def __init__(self):
        super().__init__('chain_teleop')

        #blockchain call requirements 
        self.chain_url = self.declare_parameter('substrate_url', 'ws://127.0.0.1:9944').value
        self.contract_address = self.declare_parameter('contract_address', '5FHneW46xGXgs5mUiveU4sbTyGBzmst71kXZ3BuZBaKh7nHQ').value
        self.metadata = os.path.join(os.path.dirname('blockchain_controller'),'metadata','teleop_chain_controller.json')

        self.substrate = SubstrateInterface(self.chain_url)
        self.keypair = Keypair.create_from_uri('//Alice')

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.twist = Twist()
        self.create_timer(1.0, self.controller)

    def controller(self):

        contract = ContractInstance.create_from_address(
                self.contract_address,
                self.metadata,
                self.substrate )
        
        result = contract.read(self.keypair, 'linear')
        print('Current value of "get":', result.contract_result_data)


        linear = [0,0,0]
        angular = [0,0,0]
        #read the state
        self.twist.linear.x = linear[0]
        self.twist.linear.y = linear[1]
        self.twist.linear.z = linear[2]
        self.twist.angular.x = angular[0]
        self.twist.angular.y = angular[1]
        self.twist.angular.z = angular[2]
        #publish the value
        self.publisher_.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    node = ChainTeleop()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

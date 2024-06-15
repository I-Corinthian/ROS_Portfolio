#!/use/bin/env python3
import rclpy
import os
from rclpy.node import Node
from geometry_msgs.msg import Twist
from substrateinterface import SubstrateInterface, Keypair
from substrateinterface.contracts import ContractInstance, ContractCode
import argparse

class ChainTeleop(Node):

    def __init__(self,contract_address):
        super().__init__('chain_teleop')

        #blockchain call requirements 
        self.chain_url = "ws://127.0.0.1:9944"
        self.contract_address = contract_address
        self.metadata = os.path.join(os.path.dirname(__file__),'metadata','teleop_chain_controller.json')

        self.substrate = SubstrateInterface(
            url=self.chain_url)
        self.keypair = Keypair.create_from_uri('//Alice')

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.twist = Twist()
        self.create_timer(1.0, self.controller)

    def controller(self):

        contract = ContractInstance.create_from_address(
            contract_address=self.contract_address,
            metadata_file=self.metadata,
            substrate=self.substrate,
        )

        linear_result = contract.read(self.keypair,'get_linear')
        angular_result = contract.read(self.keypair,'get_angular')
        linear_state = linear_result.contract_result_data.value['Ok']
        angular_state = angular_result.contract_result_data.value['Ok']
        self.get_logger().info('Linear State: %s' % str(linear_state))
        self.get_logger().info('angular State: %s' % str(angular_state))

        #read the state
        self.twist.linear.x = float(linear_state[0])
        self.twist.linear.y = float(linear_state[1])
        self.twist.linear.z = float(linear_state[2])
        self.twist.angular.x = float(angular_state[0])
        self.twist.angular.y = float(angular_state[1])
        self.twist.angular.z = float(angular_state[2])
        #publish the value
        self.publisher_.publish(self.twist)

def main(args=None):
    parser = argparse.ArgumentParser(description='ROS2 node to interact with TeleopChainController Ink! contract')
    parser.add_argument('--contract_address', type=str, required=True, help='The contract address of the TeleopChainController Ink! contract')
    cli_args = parser.parse_args()
    rclpy.init(args=args)
    node = ChainTeleop(contract_address=cli_args.contract_address)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

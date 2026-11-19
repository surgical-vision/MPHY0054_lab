#!/usr/bin/env python3
import numpy as np
import PyKDL as kdl
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from lab_support import Kinematics, checked_array, joint_values, to_kdl, to_numpy, transform_message


class YoubotJacobianNode(Node):
    def __init__(self):
        super().__init__('jacobian', namespace='lab06_task')
        self.declare_parameter('robot_description', '')
        self.model = Kinematics(self.get_parameter('robot_description').value)
        self.br = TransformBroadcaster(self)
        self.sub_fk = self.create_subscription(JointState, 'joint_states', self.publish_fk, 10)
        ############################################################################
        # BEGIN STUDENT CODE: jacobian_subscription



        raise NotImplementedError("Complete jacobian_subscription")



        # END STUDENT CODE: jacobian_subscription
        ############################################################################
        assert self.sub_jac in self.subscriptions
        assert self.sub_jac.topic_name == self.get_namespace().rstrip('/') + '/joint_states'
        ############################################################################
        # BEGIN STUDENT CODE: jacobian_solver



        raise NotImplementedError("Complete jacobian_solver")



        # END STUDENT CODE: jacobian_solver
        ############################################################################
        assert isinstance(self.jac_calc, kdl.ChainJntToJacSolver)
        self.last_jacobian = None

    def publish_fk(self, message):
        values = joint_values(message)
        if values is not None:
            self.br.sendTransform(transform_message(self.model.frame(values), 'urdf_link', message.header.stamp))

    def print_jacobian(self, message):
        """Select the five arm joints by name and assign jacobian, shape (6,5)."""
        if joint_values(message) is None:
            return
        ############################################################################
        # BEGIN STUDENT CODE: print_jacobian



        raise NotImplementedError("Complete print_jacobian")



        # END STUDENT CODE: print_jacobian
        ############################################################################
        checked_array(jacobian, (6, 5))
        if self.last_jacobian is None or not np.allclose(jacobian, self.last_jacobian):
            self.get_logger().info('Jacobian [linear; angular], base_link coordinates:\n' + str(jacobian))
        self.last_jacobian = jacobian.copy()

    def get_jacobian(self, joint_readings):
        """Input: five angles in radians. Assign a base-frame geometric Jacobian (6,5)."""
        checked_array(np.asarray(joint_readings), (5,))
        ############################################################################
        # BEGIN STUDENT CODE: get_jacobian



        raise NotImplementedError("Complete get_jacobian")



        # END STUDENT CODE: get_jacobian
        ############################################################################
        checked_array(jacobian, (6, 5))
        return jacobian



############################################################################
# BEGIN STUDENT CODE: helpers






# END STUDENT CODE: helpers
############################################################################


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = YoubotJacobianNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

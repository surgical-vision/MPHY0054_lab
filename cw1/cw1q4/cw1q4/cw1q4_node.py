#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import numpy as np
from std_msgs.msg import Float64

############################################################################
# BEGIN STUDENT CODE: service_imports






# END STUDENT CODE: service_imports
############################################################################


class QuatToEulerService(Node):
    def __init__(self):
        super().__init__('quat_to_euler_service_node', start_parameter_services=False)
        self.srv = self.create_service(QuatToEuler, 'quat_to_euler', self.quat_to_euler_callback)

    def quat_to_euler_callback(self, request, response):
        q_x, q_y, q_z, q_w = request.q.x, request.q.y, request.q.z, request.q.w
        self.get_logger().info(f'Euler service received quaternion: [w={q_w}, x={q_x}, y={q_y}, z={q_z}]')
        response.z, response.y, response.x = Float64(), Float64(), Float64()
        ############################################################################
        # BEGIN STUDENT CODE: quaternion_to_euler



        raise NotImplementedError("Complete quaternion_to_euler inside its student region")



        # END STUDENT CODE: quaternion_to_euler
        ############################################################################
        assert all(isinstance(value, Float64) for value in (response.x, response.y, response.z)), "Return three std_msgs/Float64 fields"
        assert np.all(np.isfinite([response.x.data, response.y.data, response.z.data])), "Conversion output must be finite"

        self.get_logger().info(f'Calculated Euler Angles: [z={response.z.data}, y={response.y.data}, x={response.x.data}]')
        return response

class QuatToRodriguesService(Node):
    def __init__(self):
        super().__init__('quat_to_rodrigues_service_node', start_parameter_services=False)
        self.srv = self.create_service(QuatToRodrigues, 'quat_to_rodrigues', self.quat_to_rodrigues_callback)

    def quat_to_rodrigues_callback(self, request, response):
        q_x, q_y, q_z, q_w = request.q.x, request.q.y, request.q.z, request.q.w
        self.get_logger().info(f'Rodrigues service received quaternion: [w={q_w}, x={q_x}, y={q_y}, z={q_z}]')
        response.x, response.y, response.z = Float64(), Float64(), Float64()
        ############################################################################
        # BEGIN STUDENT CODE: quaternion_to_rodrigues



        raise NotImplementedError("Complete quaternion_to_rodrigues inside its student region")



        # END STUDENT CODE: quaternion_to_rodrigues
        ############################################################################
        assert all(isinstance(value, Float64) for value in (response.x, response.y, response.z)), "Return three std_msgs/Float64 fields"
        assert np.all(np.isfinite([response.x.data, response.y.data, response.z.data])), "Conversion output must be finite"

        self.get_logger().info(f'Calculated Rodrigues Vector: [x={response.x.data}, y={response.y.data}, z={response.z.data}]')
        return response

def main(args=None):
    # Q4a: check the generated service schemas before starting either service.
    for service, fields in ((QuatToEuler, ('z', 'y', 'x')), (QuatToRodrigues, ('x', 'y', 'z'))):
        assert service.Request.get_fields_and_field_types() == {'q': 'geometry_msgs/Quaternion'}, "Q4a: request must contain geometry_msgs/Quaternion q"
        assert service.Response.get_fields_and_field_types() == {field: 'std_msgs/Float64' for field in fields}, "Q4a: response must contain three std_msgs/Float64 fields"
    rclpy.init(args=args)
    try:
        quat_to_euler_node = QuatToEulerService()
        quat_to_rodrigues_node = QuatToRodriguesService()

        # Use a MultiThreadedExecutor to handle callbacks from both services concurrently
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(quat_to_euler_node)
        executor.add_node(quat_to_rodrigues_node)

        print("Both Quaternion conversion services are ready.")

        try:
            executor.spin()
        finally:
            executor.shutdown()
            quat_to_euler_node.destroy_node()
            quat_to_rodrigues_node.destroy_node()
    finally:
        rclpy.shutdown()


############################################################################
# BEGIN STUDENT CODE: helpers






# END STUDENT CODE: helpers
############################################################################

if __name__ == '__main__':
    main()

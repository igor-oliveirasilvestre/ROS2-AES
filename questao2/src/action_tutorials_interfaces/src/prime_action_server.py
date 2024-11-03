import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from action_tutorials_interfaces.action import Prime


def is_prime(n):
    if n <= 1:
        return False
    for i in range(2, int(n**0.5) + 1):
        if n % i == 0:
            return False
    return True


def next_n_primes(start, n):
    primes = []
    candidate = start + 1  # Start searching from the number just after `start`
    while len(primes) < n:
        if is_prime(candidate):
            primes.append(candidate)
        candidate += 1
    return primes


class PrimeActionServer(Node):

    def __init__(self):
        super().__init__('prime_action_server')
        self._action_server = ActionServer(
            self,
            Prime,
            'prime',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Prime.Feedback()
        primes_sequence = next_n_primes(goal_handle.request.start, 10)
        feedback_msg.partial_sequence = []

        # Provide feedback after each prime number is found
        for prime in primes_sequence:
            feedback_msg.partial_sequence.append(prime)
            self.get_logger().info(f'Feedback: {feedback_msg.partial_sequence}')
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)  # simulate delay for each feedback

        goal_handle.succeed()

        result = Prime.Result()
        result.sequence = feedback_msg.partial_sequence
        return result


def main(args=None):
    rclpy.init(args=args)

    prime_action_server = PrimeActionServer()

    rclpy.spin(prime_action_server)


if __name__ == '__main__':
    main()

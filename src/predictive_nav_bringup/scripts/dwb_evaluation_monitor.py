#!/usr/bin/env python3
"""Print one DWB candidate-evaluation message for corner-stall diagnosis.

This node is intentionally diagnostic-only: it subscribes once to the DWB
evaluation topic and never publishes velocity, goals, TF, or parameters.
"""

import math

import rclpy
from dwb_msgs.msg import LocalPlanEvaluation
from rclpy.node import Node


class DwbEvaluationMonitor(Node):
    def __init__(self):
        super().__init__("dwb_evaluation_monitor")
        self.declare_parameter("topic", "/controller_server/evaluation")
        self.received = False
        topic = self.get_parameter("topic").value
        self.subscription = self.create_subscription(
            LocalPlanEvaluation, topic, self.on_evaluation, 10
        )
        self.get_logger().info(
            f"Waiting for one DWB evaluation message on {topic} ..."
        )

    def on_evaluation(self, message: LocalPlanEvaluation):
        if self.received:
            return
        self.received = True

        count = len(message.twists)
        if count == 0:
            self.get_logger().warn("DWB evaluation contains no trajectories.")
            rclpy.shutdown()
            return

        best_index = message.best_index
        if best_index >= count:
            self.get_logger().error(
                f"Invalid best_index={best_index}; trajectory count={count}."
            )
            rclpy.shutdown()
            return

        best = message.twists[best_index]
        velocity = best.traj.velocity
        self.get_logger().info(
            "DWB evaluation: "
            f"candidates={count}, best_index={best_index}, "
            f"best_velocity=(x={velocity.x:.3f} m/s, theta={velocity.theta:.3f} rad/s), "
            f"total={best.total:.3f}"
        )

        forward = [
            score for score in message.twists
            if score.traj.velocity.x > 0.01 and math.isfinite(score.total)
        ]
        self.get_logger().info(
            f"Finite-score forward candidates (x > 0.01 m/s): {len(forward)}/{count}"
        )
        nonnegative_forward = [score for score in forward if score.total >= 0.0]
        self.get_logger().info(
            "Nonnegative-total forward candidates: "
            f"{len(nonnegative_forward)}/{len(forward)}"
        )

        self.print_scores("Best overall trajectory", best)

        if forward:
            self.print_scores(
                "Lowest-total forward trajectory (finite score)",
                min(forward, key=lambda score: score.total),
            )
        if nonnegative_forward:
            self.print_scores(
                "Lowest-total nonnegative forward trajectory",
                min(nonnegative_forward, key=lambda score: score.total),
            )

        self.destroy_subscription(self.subscription)
        self.create_timer(0.1, self.shutdown_once)

    def print_scores(self, label, trajectory_score):
        velocity = trajectory_score.traj.velocity
        self.get_logger().info(
            f"{label}: x={velocity.x:.3f} m/s, "
            f"theta={velocity.theta:.3f} rad/s, total={trajectory_score.total:.3f}"
        )
        for score in trajectory_score.scores:
            contribution = score.raw_score * score.scale
            self.get_logger().info(
                f"  {score.name}: raw={score.raw_score:.3f}, "
                f"scale={score.scale:.3f}, weighted={contribution:.3f}"
            )

    def shutdown_once(self):
        self.destroy_node()
        rclpy.shutdown()


def main():
    rclpy.init()
    node = DwbEvaluationMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()

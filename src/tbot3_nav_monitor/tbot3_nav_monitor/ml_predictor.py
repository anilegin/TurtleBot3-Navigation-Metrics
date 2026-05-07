from collections import deque
from pathlib import Path
import os
import pickle

import numpy as np
import torch
from torch import nn
import joblib

import rclpy
from rclpy.node import Node

from tbot3_nav_monitor_msgs.msg import NavigationMetrics
from tbot3_nav_monitor_msgs.msg import MLNavigationPrediction


class NavigationLSTM(nn.Module):

    def __init__(self, input_size, hidden_size=64, num_layers=2, dropout=0.2):
        super().__init__()

        self.lstm = nn.LSTM(
            input_size=input_size,
            hidden_size=hidden_size,
            num_layers=num_layers,
            batch_first=True,
            dropout=dropout
        )

        self.classifier = nn.Sequential(
            nn.Linear(hidden_size, 32),
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(32, 2)
        )

    def forward(self, x):
        _, (hidden, _) = self.lstm(x)
        last_hidden = hidden[-1]
        return self.classifier(last_hidden)


class MLPredictor(Node):

    def __init__(self):
        super().__init__('ml_predictor')

        self.window_size = self.declare_parameter(
            'window_size',
            8
        ).value

        self.risk_threshold = self.declare_parameter(
            'risk_threshold',
            0.95
        ).value

        models_dir = Path('/root/tbot3_ws/models')

        self.model_path = str(models_dir / 'navigation_lstm_model.pth')
        self.scaler_path = str(models_dir / 'navigation_scaler.pkl')
        self.features_path = str(models_dir / 'feature_columns.pkl')

        self.device = torch.device(
            'cuda' if torch.cuda.is_available() else 'cpu'
        )

        self.feature_cols = joblib.load(self.features_path)
        self.scaler = joblib.load(self.scaler_path)

        self.model = NavigationLSTM(
            input_size=len(self.feature_cols),
            hidden_size=64,
            num_layers=2,
            dropout=0.2
        )

        self.model.load_state_dict(
            torch.load(self.model_path, map_location=self.device)
        )

        self.model.to(self.device)
        self.model.eval()

        self.sequence_buffer = deque(maxlen=self.window_size)

        self.prediction_pub = self.create_publisher(
            MLNavigationPrediction,
            '/ml_navigation_prediction',
            10
        )

        self.create_subscription(
            NavigationMetrics,
            '/navigation_metrics',
            self.metrics_callback,
            10
        )

        self.get_logger().info('ML Predictor node started.')

    def metrics_to_features(self, msg):
        values = {
            "total_distance": msg.total_distance,
            "current_x": msg.current_x,
            "current_y": msg.current_y,
            "goal_x": msg.goal_x,
            "goal_y": msg.goal_y,
            "commanded_speed": msg.commanded_speed,
            "actual_speed": msg.actual_speed,
            "speed_error": msg.speed_error,
            "closest_obstacle_distance": msg.closest_obstacle_distance,
            "mean_obstacle_distance": msg.mean_obstacle_distance,
            "obstacle_density": msg.obstacle_density,
            "goal_progress_rate": msg.goal_progress_rate,
            "environment_complexity": msg.environment_complexity,
            "battery_consumption": msg.battery_consumption,
            "navigation_accuracydistance2goal": msg.navigation_accuracy,
            "obstacle_avoidance_efficiency": msg.obstacle_avoidance_efficiency,
            "left_clearance": msg.left_clearance,
            "right_clearance": msg.right_clearance,
            "front_clearance": msg.front_clearance,
            "corridor_score": msg.corridor_score,
            "optimal_path_length": msg.optimal_path_length,
            "path_execution_time": msg.path_execution_time,
        }

        feature_vector = []

        for col in self.feature_cols:
            feature_vector.append(float(values.get(col, 0.0)))

        feature_vector = np.array(feature_vector, dtype=np.float32)

        feature_vector = np.nan_to_num(
            feature_vector,
            nan=0.0,
            posinf=10.0,
            neginf=0.0
        )

        feature_vector = np.clip(feature_vector, -100.0, 100.0)

        return feature_vector.tolist()

    def metrics_callback(self, msg):
        feature_vector = self.metrics_to_features(msg)
        self.sequence_buffer.append(feature_vector)

        if len(self.sequence_buffer) < self.window_size:
            return

        x = np.array(self.sequence_buffer, dtype=np.float32)

        x_scaled = self.scaler.transform(x)
        x_scaled = x_scaled.reshape(1, self.window_size, len(self.feature_cols))

        x_tensor = torch.tensor(
            x_scaled,
            dtype=torch.float32
        ).to(self.device)

        with torch.no_grad():
            logits = self.model(x_tensor)
            probs = torch.softmax(logits, dim=1)
            risk_prob = probs[0, 1].item()

        pred_msg = MLNavigationPrediction()
        pred_msg.header.stamp = self.get_clock().now().to_msg()
        pred_msg.header.frame_id = 'map'

        pred_msg.risk_probability = float(risk_prob)
        pred_msg.predicted_risky_navigation = bool(
            risk_prob >= self.risk_threshold
        )

        self.prediction_pub.publish(pred_msg)

        self.get_logger().info(
            f'ML prediction | goal_id={msg.goal_id} | '
            f'risk_prob={risk_prob:.3f} | '
            f'predicted_risky={pred_msg.predicted_risky_navigation}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = MLPredictor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
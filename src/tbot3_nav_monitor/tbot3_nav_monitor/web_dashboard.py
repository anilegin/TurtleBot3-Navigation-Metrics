import math
import threading

import rclpy
from rclpy.node import Node

from flask import Flask, jsonify, render_template_string

from tbot3_nav_monitor_msgs.msg import NavigationMetrics, MLNavigationPrediction


app = Flask(__name__)


latest_data = {
    "navigation_status": {
        "mode": "WAITING",
        "goal_id": "N/A",
        "goal_reached": "N/A",
        "execution_time": "N/A",
    },
    "ml_navigation_risk": {
        "prediction_goal_id": "N/A",
        "risk_probability": "N/A",
        "predicted_risky": "N/A",
    },
    "robot_motion": {
        "position": "N/A",
        "commanded_speed": "N/A",
        "actual_speed": "N/A",
        "speed_error": "N/A",
        "progress_rate": "N/A",
    },
    "goal_tracking": {
        "goal": "N/A",
        "distance_to_goal": "N/A",
        "total_path_length": "N/A",
        "optimal_path_length": "N/A",
        "path_efficiency_ratio": "N/A",
    },
    "obstacle_environment": {
        "closest_obstacle": "N/A",
        "mean_obstacle_distance": "N/A",
        "obstacle_density": "N/A",
        "environment_complexity": "N/A",
    },
    "narrow_passage": {
        "left_clearance": "N/A",
        "right_clearance": "N/A",
        "front_clearance": "N/A",
        "corridor_score": "N/A",
    },
    "simulated_battery": {
        "battery_used": "N/A",
        "stuck_events": "N/A",
    },
}


class WebDashboardNode(Node):

    def __init__(self):
        super().__init__("web_dashboard")

        self.latest_risk_probability = None
        self.latest_predicted_risky_navigation = None
        self.latest_prediction_goal_id = None

        self.create_subscription(
            NavigationMetrics,
            "/navigation_metrics",
            self.metrics_callback,
            10
        )

        self.create_subscription(
            MLNavigationPrediction,
            "/ml_navigation_prediction",
            self.prediction_callback,
            10
        )

        self.get_logger().info("Web dashboard running at http://localhost:5000")

    def safe_float(self, value, digits=2, suffix=""):
        if value is None:
            return "N/A"

        if math.isnan(value) or math.isinf(value):
            return "N/A"

        return f"{value:.{digits}f}{suffix}"

    def prediction_callback(self, msg):
        self.latest_risk_probability = msg.risk_probability
        self.latest_predicted_risky_navigation = msg.predicted_risky_navigation
        self.latest_prediction_goal_id = msg.goal_id

    def get_navigation_mode(self, msg):
        if msg.goal_reached:
            return "GOAL REACHED"

        if msg.navigation_status == 2:
            return "STUCK"

        if (
            self.latest_predicted_risky_navigation is True
            and self.latest_prediction_goal_id == msg.goal_id
        ):
            return "ML HIGH RISK"

        if msg.corridor_score > 0.5:
            return "NARROW PASSAGE"

        if msg.environment_complexity > 0.8:
            return "COMPLEX ENVIRONMENT"

        return "NORMAL"

    def metrics_callback(self, msg):
        mode = self.get_navigation_mode(msg)

        latest_data["navigation_status"] = {
            "mode": mode,
            "goal_id": msg.goal_id,
            "goal_reached": msg.goal_reached,
            "execution_time": self.safe_float(msg.path_execution_time, 1, " s"),
        }

        latest_data["ml_navigation_risk"] = {
            "prediction_goal_id": (
                "N/A" if self.latest_prediction_goal_id is None
                else self.latest_prediction_goal_id
            ),
            "risk_probability": (
                "N/A" if self.latest_risk_probability is None
                else self.safe_float(self.latest_risk_probability, 2)
            ),
            "predicted_risky": (
                "N/A" if self.latest_predicted_risky_navigation is None
                else self.latest_predicted_risky_navigation
            ),
        }

        latest_data["robot_motion"] = {
            "position": (
                f"x={self.safe_float(msg.current_x, 2)}, "
                f"y={self.safe_float(msg.current_y, 2)}"
            ),
            "commanded_speed": self.safe_float(msg.commanded_speed, 2, " m/s"),
            "actual_speed": self.safe_float(msg.actual_speed, 2, " m/s"),
            "speed_error": self.safe_float(msg.speed_error, 2, " m/s"),
            "progress_rate": self.safe_float(msg.goal_progress_rate, 4, " m/update"),
        }

        latest_data["goal_tracking"] = {
            "goal": (
                f"x={self.safe_float(msg.goal_x, 2)}, "
                f"y={self.safe_float(msg.goal_y, 2)}"
            ),
            "distance_to_goal": self.safe_float(msg.navigation_accuracy, 2, " m"),
            "total_path_length": self.safe_float(msg.total_distance, 2, " m"),
            "optimal_path_length": self.safe_float(msg.optimal_path_length, 2, " m"),
            "path_efficiency_ratio": self.safe_float(
                msg.obstacle_avoidance_efficiency,
                2
            ),
        }

        latest_data["obstacle_environment"] = {
            "closest_obstacle": self.safe_float(
                msg.closest_obstacle_distance,
                2,
                " m"
            ),
            "mean_obstacle_distance": self.safe_float(
                msg.mean_obstacle_distance,
                2,
                " m"
            ),
            "obstacle_density": self.safe_float(msg.obstacle_density, 2),
            "environment_complexity": self.safe_float(
                msg.environment_complexity,
                2
            ),
        }

        latest_data["narrow_passage"] = {
            "left_clearance": self.safe_float(msg.left_clearance, 2, " m"),
            "right_clearance": self.safe_float(msg.right_clearance, 2, " m"),
            "front_clearance": self.safe_float(msg.front_clearance, 2, " m"),
            "corridor_score": self.safe_float(msg.corridor_score, 1),
        }

        latest_data["simulated_battery"] = {
            "battery_used": self.safe_float(msg.battery_consumption, 2, "%"),
            "stuck_events": msg.stuck_count,
        }


HTML = """
<!DOCTYPE html>
<html>
<head>
    <title>TurtleBot3 Navigation Monitor</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            background: #111827;
            color: #f9fafb;
            padding: 24px;
        }

        h1 {
            margin-bottom: 8px;
        }

        .subtitle {
            color: #9ca3af;
            margin-bottom: 24px;
        }

        .grid {
            display: grid;
            grid-template-columns: repeat(2, minmax(360px, 1fr));
            gap: 16px;
        }

        .card {
            background: #1f2937;
            border-radius: 14px;
            padding: 18px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.35);
        }

        .card h2 {
            margin-top: 0;
            font-size: 18px;
            color: #38bdf8;
            border-bottom: 1px solid #374151;
            padding-bottom: 8px;
        }

        table {
            width: 100%;
            border-collapse: collapse;
        }

        td {
            padding: 7px 0;
            border-bottom: 1px solid #374151;
        }

        td:first-child {
            color: #9ca3af;
            text-transform: uppercase;
            font-size: 12px;
            letter-spacing: 0.04em;
        }

        td:last-child {
            text-align: right;
            font-weight: bold;
            font-size: 16px;
        }

        .mode-normal {
            color: #22c55e;
        }

        .mode-risk {
            color: #f472b6;
        }

        .mode-stuck {
            color: #ef4444;
        }

        .mode-goal {
            color: #38bdf8;
        }
    </style>
</head>
<body>
    <h1>TurtleBot3 Navigation Monitor</h1>
    <div class="subtitle">Live ROS2 navigation metrics and ML risk prediction</div>

    <div class="grid" id="dashboard"></div>

    <script>
        function titleCase(text) {
            return text
                .replaceAll("_", " ")
                .replace(/\\b\\w/g, c => c.toUpperCase());
        }

        function modeClass(value) {
            if (value === "ML HIGH RISK") return "mode-risk";
            if (value === "STUCK") return "mode-stuck";
            if (value === "GOAL REACHED") return "mode-goal";
            return "mode-normal";
        }

        async function updateDashboard() {
            const res = await fetch("/data");
            const data = await res.json();

            const dashboard = document.getElementById("dashboard");
            dashboard.innerHTML = "";

            for (const [section, values] of Object.entries(data)) {
                const card = document.createElement("div");
                card.className = "card";

                const title = document.createElement("h2");
                title.innerText = titleCase(section);
                card.appendChild(title);

                const table = document.createElement("table");

                for (const [key, value] of Object.entries(values)) {
                    const row = document.createElement("tr");

                    const keyCell = document.createElement("td");
                    keyCell.innerText = titleCase(key);

                    const valueCell = document.createElement("td");
                    valueCell.innerText = value;

                    if (key === "mode") {
                        valueCell.className = modeClass(value);
                    }

                    row.appendChild(keyCell);
                    row.appendChild(valueCell);
                    table.appendChild(row);
                }

                card.appendChild(table);
                dashboard.appendChild(card);
            }
        }

        setInterval(updateDashboard, 500);
        updateDashboard();
    </script>
</body>
</html>
"""


@app.route("/")
def index():
    return render_template_string(HTML)


@app.route("/data")
def data():
    return jsonify(latest_data)


def ros_spin():
    rclpy.init()
    node = WebDashboardNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


def main():
    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()
    app.run(host="0.0.0.0", port=5000, debug=False)


if __name__ == "__main__":
    main()
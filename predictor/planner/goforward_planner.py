# Go forward planner - dummy proof of concept

class GoFowardPlanner(Planner):
    """Dummy planner that sends a simple Go forward message every time. """

    def __init__(self):
        self.traj = None

    def on_env(self, map_name,
            my_traj: Trajectory,
            human_input: []
            other_trajs: [],
            obstacle_info: [],
            reference_path: Trajectory,
            ):
        self.traj = my_traj

    def fetch_my_plan(self) -> Trajectory:
        """Retrieve my trajectory"""
        return self.traj

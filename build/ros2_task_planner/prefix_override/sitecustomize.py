import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/popem/spring-2025-final-project-team-4/install/ros2_task_planner'

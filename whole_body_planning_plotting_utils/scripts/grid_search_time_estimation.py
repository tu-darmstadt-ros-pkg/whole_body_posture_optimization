import math

time_per_estimation = 1.0
joints = 4
upper_limit = math.pi/2.0
lower_limit = -math.pi/2.0
resolution = 0.35 # math.radians(20)
trajectory_length = 2.5
trajectory_resolution = 0.05

def main():
    print("Resolution:", resolution)
    joint_range = upper_limit - lower_limit
    steps_per_joint = joint_range / resolution
    steps_all_joints = steps_per_joint ** joints
    time_per_position = time_per_estimation * steps_all_joints
    print_time(time_per_position)

    trajectory_steps = trajectory_length / trajectory_resolution
    time_per_trajectory = trajectory_steps * time_per_position
    print_time(time_per_trajectory)

def print_time(time_in_ms):
    if time_in_ms < 1000.0:
        print(time_in_ms, "ms")
    elif time_in_ms < 1000.0 * 60.0:
        print(time_in_ms / 1000.0, "sec")
    elif time_in_ms < 1000.0 * 60 * 60:
        print(time_in_ms / 1000.0 / 60.0, "min")
    elif time_in_ms < 1000.0 * 60 * 60 * 24:
        print(time_in_ms / 1000.0 / 60.0 / 60.0, "h")
    else:
        print(time_in_ms / 1000.0 / 60.0 / 60 / 24.0, " days")


if __name__ == "__main__":
    main()


"""Removed legacy UWB + lidar grid/A* entry.

The current test path uses APP.test_lidar_track with direct OpenArt priority,
lidar fallback tracking, and no occupancy grid.
"""


def main():
    print("test_uwb_two_anchor_localization has been removed: lidar grid/A* is no longer used.")
    print("Run APP/test_lidar_track.py or set APP_PROFILE = 'test_lidar_track'.")


if __name__ == "__main__":
    main()

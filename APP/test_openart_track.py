"""Compatibility entry: run lidar object tracking.

This file used to start OpenArt tracking. The current project path uses
YDLIDAR tracking, so running this legacy entry from the IDE now delegates to
APP.test_lidar_track.
"""

from APP.test_lidar_track import main


if __name__ == "__main__":
    main()

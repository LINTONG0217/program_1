"""APP 层统一入口。"""

from APP.run import run_profile
from Module import config


def main():
    profile = getattr(config, "APP_PROFILE", "single")
    print("APP profile:", profile)
    run_profile(profile)


if __name__ == "__main__":
    main()

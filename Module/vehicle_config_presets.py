"""Vehicle-specific config presets.

Usage from Module/config.py:
	set VEHICLE_PRESET to one of:
	- master_test
	- slave_test
	- master_competition
	- slave_competition
"""


def _common_two_anchor():
	return {
		"UWB_POSE_ENABLE": True,
		"UWB_UART_ID": 5,
		"UWB_BAUDRATE": 115200,
		"UWB_TX_PIN": "D20",
		"UWB_RX_PIN": "D21",
		"UWB_ANCHOR_0_ID": 0,
		"UWB_ANCHOR_0_X_M": 0.0,
		"UWB_ANCHOR_0_Y_M": 0.0,
		"UWB_ANCHOR_1_ID": 1,
		"UWB_ANCHOR_1_X_M": 3.2,
		"UWB_ANCHOR_1_Y_M": 0.0,
		"UWB_TWO_ANCHOR_PREFERRED_SIDE": 1,
		"UWB_TWO_ANCHOR_INIT_ENABLE": True,
		"UWB_RANGE_POLL_ENABLE": True,
		"UWB_RANGE_POLL_MS": 120,
		"UWB_RANGE_CMD_TEMPLATE": "AT+DISTANCE\r\n",
		"UWB_TWO_ANCHOR_USE_IMU": False,
	}


PRESETS = {
	"master_test": dict(
		_common_two_anchor(),
		APP_PROFILE="test_uwb_two_anchor",
		CAR_LINK_TEST_ROLE="master",
		CAR_LINK_ROLE="master",
		COMPETITION_DUAL_ENABLE=True,
		UWB_TWO_ANCHOR_INIT_X_M=0.25,
		UWB_TWO_ANCHOR_INIT_Y_M=0.25,
		UWB_TWO_ANCHOR_INIT_YAW_DEG=0.0,
	),
	"slave_test": dict(
		_common_two_anchor(),
		APP_PROFILE="test_uwb_two_anchor",
		CAR_LINK_TEST_ROLE="slave",
		CAR_LINK_ROLE="slave",
		COMPETITION_DUAL_ENABLE=True,
		UWB_TWO_ANCHOR_INIT_X_M=0.45,
		UWB_TWO_ANCHOR_INIT_Y_M=0.25,
		UWB_TWO_ANCHOR_INIT_YAW_DEG=0.0,
	),
	"master_competition": dict(
		_common_two_anchor(),
		APP_PROFILE="competition",
		CAR_LINK_TEST_ROLE="master",
		CAR_LINK_ROLE="master",
		COMPETITION_DUAL_ENABLE=True,
		UWB_TWO_ANCHOR_INIT_X_M=0.25,
		UWB_TWO_ANCHOR_INIT_Y_M=0.25,
		UWB_TWO_ANCHOR_INIT_YAW_DEG=0.0,
	),
	"slave_competition": dict(
		_common_two_anchor(),
		APP_PROFILE="dual_slave",
		CAR_LINK_TEST_ROLE="slave",
		CAR_LINK_ROLE="slave",
		COMPETITION_DUAL_ENABLE=True,
		UWB_TWO_ANCHOR_INIT_X_M=0.45,
		UWB_TWO_ANCHOR_INIT_Y_M=0.25,
		UWB_TWO_ANCHOR_INIT_YAW_DEG=0.0,
	),
}


def apply_preset(target_globals, preset_name):
	name = str(preset_name or "").strip().lower()
	if not name or name == "manual":
		return False
	values = PRESETS.get(name)
	if not values:
		raise ValueError("unknown VEHICLE_PRESET: {}".format(preset_name))
	target_globals.update(values)
	return True

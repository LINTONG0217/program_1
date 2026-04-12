"""RM二进制协议模块。"""

try:
	import ustruct as struct
except ImportError:
	import struct


KIND_COMMAND = 1
KIND_STATUS = 2
KIND_HEARTBEAT = 3

# File transfer (used for wireless update / sync)
KIND_FILE_BEGIN = 10
KIND_FILE_PATH = 11
KIND_FILE_DATA = 12
KIND_FILE_END = 13
KIND_FILE_ACK = 14
KIND_FILE_NAK = 15

STATE_IDLE = 0
STATE_RUN = 1
STATE_PUSH = 2
STATE_STOP = 3
STATE_TIMEOUT = 4

_MAGIC = bytes((0xA5, 0x5A))
_HEADER_FMT = "<2sBBHH"
_HEADER_SIZE = struct.calcsize(_HEADER_FMT)
_CRC_FMT = "<H"
_CRC_SIZE = struct.calcsize(_CRC_FMT)


def crc16_ccitt(data, seed=0xFFFF):
	crc = seed
	for byte in data:
		crc ^= byte << 8
		for _ in range(8):
			if crc & 0x8000:
				crc = ((crc << 1) ^ 0x1021) & 0xFFFF
			else:
				crc = (crc << 1) & 0xFFFF
	return crc


def state_to_code(state):
	mapping = {
		"idle": STATE_IDLE,
		"run": STATE_RUN,
		"push": STATE_PUSH,
		"stop": STATE_STOP,
		"timeout": STATE_TIMEOUT,
	}
	return mapping.get(state, STATE_IDLE)


def code_to_state(code):
	mapping = {
		STATE_IDLE: "idle",
		STATE_RUN: "run",
		STATE_PUSH: "push",
		STATE_STOP: "stop",
		STATE_TIMEOUT: "timeout",
	}
	return mapping.get(code, "idle")


def pack_frame(kind, seq, payload, version=1):
	payload = payload or b""
	header = struct.pack(_HEADER_FMT, _MAGIC, version, kind, seq & 0xFFFF, len(payload))
	crc = crc16_ccitt(header[2:] + payload)
	return header + payload + struct.pack(_CRC_FMT, crc)


def unpack_frames(buffer, max_payload=64):
	packets = []
	start = 0
	buffer_len = len(buffer)

	while start + _HEADER_SIZE + _CRC_SIZE <= buffer_len:
		if buffer[start:start + 2] != _MAGIC:
			start += 1
			continue

		magic, version, kind, seq, length = struct.unpack(
			_HEADER_FMT,
			buffer[start:start + _HEADER_SIZE],
		)
		if magic != _MAGIC or length > max_payload:
			start += 1
			continue

		frame_size = _HEADER_SIZE + length + _CRC_SIZE
		if start + frame_size > buffer_len:
			break

		payload_start = start + _HEADER_SIZE
		payload_end = payload_start + length
		payload = buffer[payload_start:payload_end]
		recv_crc = struct.unpack(_CRC_FMT, buffer[payload_end:payload_end + _CRC_SIZE])[0]
		calc_crc = crc16_ccitt(buffer[start + 2:payload_end])
		if recv_crc == calc_crc:
			packets.append({
				"version": version,
				"kind": kind,
				"seq": seq,
				"payload": payload,
			})
			start += frame_size
		else:
			start += 1

	return packets, buffer[start:]


"""网格地图显示。"""


class GridMap:
	def __init__(self, lcd):
		self.lcd = lcd
		self.grid_res = 0.3
		self.cell_size = 18
		self.offset_x = 4
		self.offset_y = 20
		self.last_gx = -1
		self.last_gy = -1

	def draw_bg(self):
		"""开机画 10x10 的底板网格"""
		self.lcd.clear(0xFFFF)
		self.lcd.color(0xCE59, 0xFFFF)

		line_len = 10 * self.cell_size

		for i in range(11):
			x = self.offset_x + i * self.cell_size
			self.lcd.line(x, self.offset_y, x, self.offset_y + line_len)

			y = self.offset_y + i * self.cell_size
			self.lcd.line(self.offset_x, y, self.offset_x + line_len, y)

	def update_car(self, real_x, real_y):
		"""物理坐标 -> 格子坐标，并刷新屏幕"""
		gx = 9 - int(real_x / self.grid_res)
		gy = 0 + int(real_y / self.grid_res)

		gx = max(0, min(9, gx))
		gy = max(0, min(9, gy))

		if gx == self.last_gx and gy == self.last_gy:
			return

		block_size = self.cell_size - 4

		if self.last_gx != -1:
			px_old = self.offset_x + self.last_gx * self.cell_size + 2
			py_old = self.offset_y + self.last_gy * self.cell_size + 2

			self.lcd.color(0xFFFF, 0xFFFF)
			for dy in range(block_size):
				self.lcd.line(px_old, py_old + dy, px_old + block_size, py_old + dy)

		px_new = self.offset_x + gx * self.cell_size + 2
		py_new = self.offset_y + gy * self.cell_size + 2

		self.lcd.color(0x001F, 0xFFFF)
		for dy in range(block_size):
			self.lcd.line(px_new, py_new + dy, px_new + block_size, py_new + dy)

		self.last_gx = gx
		self.last_gy = gy

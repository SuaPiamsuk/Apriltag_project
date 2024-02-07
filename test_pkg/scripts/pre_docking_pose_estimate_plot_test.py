import numpy as np
import matplotlib.pyplot as plt

# กำหนดจุดที่ต้องการทำฟิต
points = np.array([[0.162, 1.067], [0.067, 1.064], [-0.027, 1.069]])

# ทำการทำฟิตเส้นตรงโดยใช้ np.polyfit
# กรณีทำฟิตเส้นตรงเพื่อหาสมการของเส้นตรง
coefficients = np.polyfit(points[:, 0], points[:, 1], 1)

# สร้างฟังก์ชันเส้นตรงจากค่าพหุคณะที่ได้
line_function = np.poly1d(coefficients)

# สร้างข้อมูลสำหรับพล็อตเส้นตรง
x_values = np.linspace(min(points[:, 0]), max(points[:, 0]), 100)
y_values = line_function(x_values)

x_min  = 0.162
x_max  = -0.027
y_min = line_function(x_min)
y_max = line_function(x_max)

slope = (y_max - y_min) / (x_max - x_min)

slope = -1/slope

mid_point_x  = (x_max + x_min)/2.0
mid_point_y  = (y_max + y_min)/2.0

y = -1.0

x = ((y - mid_point_y) / slope) + mid_point_x

print(mid_point_x - x)

plt.plot([mid_point_x, x], [mid_point_y, y], marker='o', label='pre_dock')



# พล็อตจุด
plt.scatter(points[:, 0], points[:, 1], label='ddd')

# พล็อตเส้นตรง
plt.plot(x_values, y_values, label='station')

plt.axis('square')
plt.grid()

# plt.plot([0, 0], [min(points[:, 1]), max(points[:, 1])], [0, 0], label='เส้นที่ตั้งฉาก', color='red')

# print(min(points[:, 0]))
# print(max(points[:, 0]))
# print(x_values)

# แสดงกราฟ
# plt.xlim(30, 30) 
plt.ylim(-10, 10) 
plt.legend()
plt.show()




# import matplotlib.pyplot as plt
# import numpy as np

# def plot_perpendicular_line(x1, y1, x2, y2):
#     # สร้างข้อมูลจุดบนเส้นตรง
#     x_values = [x1, x2]
#     y_values = [y1, y2]

#     # พล็อตเส้นตรง
#     plt.plot(x_values, y_values, label='Line through points')

#     # ลงทะเบียนแกน
#     plt.xlabel('X-axis')
#     plt.ylabel('Y-axis')

#     # เพิ่มป้ายกำกับ
#     plt.scatter(x_values, y_values, color='red')  # ลงทะเบียนจุด

#     # แสดงป้ายกำกับจุด
#     for i, txt in enumerate(['Point 1', 'Point 2']):
#         plt.annotate(txt, (x_values[i], y_values[i]), textcoords="offset points", xytext=(0, 10), ha='center')

#     # ความชันของเส้นตรงที่ผ่านไปที่จุดที่กำหนด
#     slope = (y2 - y1) / (x2 - x1)

#     # ความชันของเส้นตรงที่ตั้งฉาก
#     perpendicular_slope = -1 / slope

#     # จุดกึ่งกลางระหว่างจุดที่กำหนด
#     mid_point = ((x1 + x2) / 2, (y1 + y2) / 2)

#     # สร้างข้อมูลจุดบนเส้นตรงที่ตั้งฉาก
#     x_perpendicular = np.linspace(mid_point[0] - 2, mid_point[0] + 2, 100)
#     y_perpendicular = perpendicular_slope * (x_perpendicular - mid_point[0]) + mid_point[1]

#     # พล็อตเส้นตรงที่ตั้งฉาก
#     plt.plot(x_perpendicular, y_perpendicular, label='Perpendicular line')

#     # แสดงตาราง
#     plt.legend()

#     # แสดงกราฟ
#     plt.show()

# # กำหนดค่าของจุด
# x1, y1 = 1, 2
# x2, y2 = 3, 4

# # พล็อตเส้นที่ตั้งฉาก
# plot_perpendicular_line(x1, y1, x2, y2)


import sensor, image, time, math
from pyb import UART
uart = UART(3, 9600)  # UART3 对应 OpenMV 的 P4 (TX), P5 (RX)，波特率为 9600
uart.init(9600, bits=8, parity=None, stop=1)

gray_threshold = (0, 191)#灰度阈值根据实际情况可微调，排除最亮的点即可，后续程序有取反
green_threshold  = (100, 78, -32, 91, -60, 99)#此处LAB阈值红绿通用，不改程序名称了
sensor.reset() # Initialize the camera sensor.
sensor.set_pixformat(sensor.GRAYSCALE) # use GRAYSCALE.
sensor.set_framesize(sensor.QVGA) # use QVGA for speed.
sensor.skip_frames(10) # Let new settings take affect.
sensor.set_auto_whitebal(False) # turn this off.
sensor.set_vflip(True)
sensor.set_hmirror(True)

flag_find_first_rect = False
flag_find_second_rect = False
first_rect_corners = [[0,0] for _ in range(4)]
second_rect_corners = [[0,0] for _ in range(4)]
corner_centers = [[0,0] for _ in range(4)]
area = 0
data = ''
corner_normals = [ [(0,0), (0,0)] for _ in range(4) ]  # 全局变量，4个角点的(n1,n2)
corner_positions = [(0, 0)] * 4  # 保存当前四个角点的位置
corner_dirs = [(0, 0)] * 4 
d_pix = 0

init_flag = False
A4_flag = False
A4_finish = False
inner_flag = False
outer_flag = False
count = 0

clock = time.clock() # Tracks FPS.

def is_good_rectangle(r):
    approx_area = r.magnitude()
    bounding = r.rect()
    bounding_area = bounding[2] * bounding[3]
    if bounding_area == 0:
        return False
    ratio = approx_area / bounding_area
    return ratio > 0.7

def get_normal(p1, p2):
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    length = math.sqrt(dx * dx + dy * dy)
    if length == 0:
        return (0, 0)
    return (-dy / length, dx / length)

def move_point(p, n, dist):
    return (int(p[0] + n[0] * dist), int(p[1] + n[1] * dist))

def brightness(gray):  # 灰度图像直接返回灰度值
    if gray is None:
        return -1
    return gray  # 本身就是亮度

# 角点列表 corners = [(x0,y0), (x1,y1), (x2,y2), (x3,y3)]
# 需保证顺/逆时针，可直接用 find_rects 输出的顺序
def calc_corner_centers(corners, tape_mm=18, a4_mm=(210, 297)):
    def dist(p, q):
        dx, dy = q[0]-p[0], q[1]-p[1]
        return (dx*dx + dy*dy) ** 0.5
    def normalize(v):
        mag = (v[0]*v[0] + v[1]*v[1]) ** 0.5
        return (v[0]/mag, v[1]/mag)
    signed_area = sum(corners[i][0]*corners[(i+1)%4][1] -
                      corners[(i+1)%4][0]*corners[i][1] for i in range(4))
    ccw = signed_area > 0
    rot90 = (lambda v: (-v[1], v[0])) if ccw else (lambda v: (v[1], -v[0]))

    lens = [dist(corners[i], corners[(i+1)%4]) for i in range(4)]
    long_edges = sorted(lens, reverse=True)[:2]
    short_edges = sorted(lens)[:2]
    avg_long_pix = sum(long_edges) / 2.0
    avg_short_pix = sum(short_edges) / 2.0
    px_per_mm_long = avg_long_pix / float(max(a4_mm))
    px_per_mm_short = avg_short_pix / float(min(a4_mm))
    pixel_per_mm = (px_per_mm_long + px_per_mm_short) / 2.0
    d_pix = tape_mm * 0.5 * pixel_per_mm
    print("d_pix", d_pix)

    cx = sum(p[0] for p in corners)/4.0
    cy = sum(p[1] for p in corners)/4.0
    centroid = (cx, cy)

    centers = []
    global corner_positions
    global corner_normals
    global corner_dirs            # 声明使用全局变量
    
    corner_positions = corners[:]  # 复制角点坐标
    corner_normals = [ [(0,0), (0,0)] for _ in range(4) ]  # 重新初始化
    for i, A in enumerate(corners):
        B, C = corners[(i-1)%4], corners[(i+1)%4]
        print("i={}, A={}, B={}, C={}".format(i, A, B, C))
        e1 = normalize((B[1]-A[1], B[0]-A[0]))
        e2 = normalize((C[1]-A[1], C[0]-A[0]))
        n1 = normalize(rot90(e1))
        n2 = normalize(rot90(e2))
        corner_normals[i] = (n1, n2)
   
        def offset_center(n1, n2):
            P1 = (A[0]+n1[0]*d_pix, A[1]-n1[1]*d_pix)
            P2 = (A[0]+n2[0]*d_pix, A[1]-n2[1]*d_pix)
            det = e1[0]*(-e2[1]) - e1[1]*(-e2[0])
            dx, dy = P2[0]-P1[0], P2[1]-P1[1]
            t = (dx*(-e2[1]) - dy*(-e2[0])) / det
            return (P1[0]+e1[0]*t, P1[1]+e1[1]*t)
    
        vx, vy = normalize((n1[1] - n2[1], n1[0] - n2[0]))
        corner_dirs[i] = (vx, vy) 
        print("vx = {:.3f}, vy = {:.3f}".format(vx, vy))
        center = (A[0] + 1.414 * vx * d_pix, A[1] + 1.414 * vy * d_pix)
#        center = offset_center(n1, n2)
        v1 = (center[1]-A[1], center[0]-A[0])
        v2 = (centroid[1]-A[1], centroid[0]-A[0])
        if v1[1]*v2[1] + v1[0]*v2[0] < 0:          # 说明交点朝外
            center = offset_center((-n1[1], -n1[0]), (-n2[1], -n2[0]))
            print("交点朝外")
        centers.append(center)
    return [list(c) for c in centers]    

while(True):
    img = sensor.snapshot().lens_corr(strength = 1.2, zoom = 1.55)
    if uart.any():
        raw = uart.read()
        if raw:
            try:
                data = raw.decode().strip()
            except:
                continue  # 非法编码，跳过

            # 回显收到的原始数据
#            msg = "收到数据: {}\r\n".format(data)
#            uart.write(msg.encode())

            # 指令判断
            if data == "initStart":
                init_flag = True
                A4_flag = False
#                msg = "initStart收到\r\n"
#                uart.write(msg.encode())

            elif data == "initEnd":
                init_flag = False
                A4_flag = False
#                msg = "initEnd收到\r\n"
#                uart.write(msg.encode())

            elif data == "A4":
                A4_flag = True
                A4_finish = False
                init_flag = False
                outer_flag = False
                inner_flag = False
                msg = "A4收到\r\n"
                uart.write(msg.encode())
                
    for i in range(4):
        A = corner_positions[i]
        n1, n2 = corner_normals[i]
        img.draw_arrow(int(A[0]), int(A[1]),
                       int(A[0] + n1[0]*20), int(A[1] - n1[1]*20),
                       color=(255, 0, 0))
        img.draw_arrow(int(A[0]), int(A[1]),
                       int(A[0] + n2[0]*20), int(A[1] - n2[1]*20),
                       color=(0, 255, 0))
    for i in range(4):
        A = corner_positions[i]  # 当前角点位置
        vx, vy = corner_dirs[i]  # 当前方向向量
        x0, y0 = int(A[0]), int(A[1])
        x1 = int(A[0] + vx * 25)  # 放大25倍可视化
        y1 = int(A[1] + vy * 25)
        img.draw_arrow(x0, y0, x1, y1, color=(255, 0, 0))

    if init_flag:
        img.draw_circle(45, 7, 1, (0, 0, 255), thickness=2)
        img.draw_circle(270, 7, 1, (0, 0, 255), thickness=2)
        img.draw_circle(40, 233, 1, (0, 0, 255), thickness=2)
        img.draw_circle(275, 233, 1, (0, 0, 255), thickness=2)
    if A4_flag:
#        img = sensor.snapshot().lens_corr(strength = 1.2, zoom = 1.55)
        img.laplacian(1, sharpen=True)
        if not outer_flag:
            rects = img.find_rects(threshold=5000, x_gradient=20, y_gradient=20)
        else:
            rects = img.find_rects(threshold=10, x_gradient=20, y_gradient=20)
        rects = [r for r in rects if r.magnitude() > 100000]
        rects = [r for r in rects if is_good_rectangle(r)]

        if outer_flag:
            for i in range(4):
                img.draw_line(outer_rect_corners[i][0], outer_rect_corners[i][1],
                              outer_rect_corners[(i+1)%4][0], outer_rect_corners[(i+1)%4][1], color=255)
            for p in outer_rect_corners:
                img.draw_circle(p[0], p[1], 3, color=255)
            if not A4_finish:
                count += 1
                uart.write("count = {}\r\n".format(count).encode())
            else:
                for p in corner_centers:
                    img.draw_circle(int(p[0]), int(p[1]), 2, color=128)  # 画出角带中心点

            if count >= 3 and not A4_finish:
                corner_centers = calc_corner_centers(outer_rect_corners)
                msg = "corner_centers: {}\r\n".format(corner_centers)
                uart.write(msg.encode())

                for p in corner_centers:
                    img.draw_circle(int(p[0]), int(p[1]), 2, color=128)  # 画出角带中心点
                A4_finish = True

        if inner_flag:
            for i in range(4):
                img.draw_line(inner_rect_corners[i][0], inner_rect_corners[i][1],
                              inner_rect_corners[(i+1)%4][0], inner_rect_corners[(i+1)%4][1], color=200)
            for p in inner_rect_corners:
                img.draw_circle(p[0], p[1], 3, color=200)

        if outer_flag and inner_flag and not A4_finish:
#            msg = "outer corners: {}\r\n".format(outer_rect_corners)
#            uart.write(msg.encode())

#            msg = "inner corners: {}\r\n".format(inner_rect_corners)
#            uart.write(msg.encode())

            def sort_corners(corners):
                # 根据 y 先分上下，再根据 x 分左右
                corners = sorted(corners, key=lambda p: (p[1], p[0]))
                top = sorted(corners[:2], key=lambda p: p[0])   # top-left, top-right
                bottom = sorted(corners[2:], key=lambda p: p[0]) # bottom-left, bottom-right
                return [top[0], top[1], bottom[1], bottom[0]]  # 左上，右上，右下，左下
            
            outer_sorted = sort_corners(outer_rect_corners)
            inner_sorted = sort_corners(inner_rect_corners)
            
            corner_centers_list = []
            for i in range(4):
                x_out, y_out = outer_sorted[i]
                x_in, y_in = inner_sorted[i]
                mid_x = (x_out + x_in) / 2
                mid_y = (y_out + y_in) / 2
            
                corner_centers[i][0] = mid_x
                corner_centers[i][1] = mid_y
                corner_centers_list.append([round(mid_x, 4), round(mid_y, 4)])
                img.draw_cross(int(mid_x), int(mid_y), color=100)                
            
            msg = "corner_centers: {}\r\n".format(corner_centers_list)
            uart.write(msg.encode())
                
                
            A4_finish = True

        if not rects or A4_finish:
            continue

        if len(rects) >= 2:
            rects = sorted(rects, key=lambda r: r.magnitude(), reverse=True)

        r = rects[0]
        corners = r.corners()
        for i in range(4):
            p1 = corners[i]
            p2 = corners[(i + 1) % 4]
            mid = ((p1[0] + p2[0]) // 2, (p1[1] + p2[1]) // 2)
            normal = get_normal(p1, p2)

            inner_p = move_point(mid, normal, -4)
            outer_p = move_point(mid, normal, 4)

            if 0 <= inner_p[0] < 320 and 0 <= inner_p[1] < 240:
                inner_color = img.get_pixel(inner_p[0], inner_p[1])
                img.draw_circle(inner_p[0], inner_p[1], 2, color=150)
            else:
                inner_color = None

            if 0 <= outer_p[0] < 320 and 0 <= outer_p[1] < 240:
                outer_color = img.get_pixel(outer_p[0], outer_p[1])
                img.draw_circle(outer_p[0], outer_p[1], 2, color=100)
            else:
                outer_color = None

        b_in = brightness(inner_color)
        b_out = brightness(outer_color)

        if b_in < b_out:
            ling_type = "外框"
            outer_flag = True
            outer_rect = rects[0]
            outer_rect_corners = outer_rect.corners()
        else:
            ling_type = "内框"
            inner_flag = True
            inner_rect = rects[0]
            inner_rect_corners = inner_rect.corners()

#        uart.write("ling_type", ling_type)
        for i in range(4):
            img.draw_line(corners[i][0], corners[i][1],
                          corners[(i+1)%4][0], corners[(i+1)%4][1], color=128)
        for p in corners:
            img.draw_circle(p[0], p[1], 3, color=128)

    min_size=400
#    for i in range (1000):
    img = sensor.snapshot().lens_corr(strength = 1.2, zoom = 1.55)
    blobs = img.find_blobs([gray_threshold],x_stride=1,y_stride=1,invert=True)
#        if i > 10 :
    for blob in blobs:
        #尽量过滤掉一些杂散的blobs
        if (blob[2]*blob[3] < min_size) and (blob[2]>2) and (blob[3]>2) and (blob[2]<2*blob[3]) and (blob[3]<2*blob[2]):
#确保色块较小且较方（圆）
            yuandian = blob
            img.draw_rectangle(yuandian.rect(),color = (0,0,0)) # rect
            img.draw_cross(yuandian.cx(), yuandian.cy()) # cx, cyj v
            msg = "cx: {}, cy: {}, w: {}, h: {}\r\n".format(
            yuandian.cx(), yuandian.cy(), blob[2], blob[3])
            uart.write(msg.encode())


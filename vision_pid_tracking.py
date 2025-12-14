import cv2
import numpy as np
import time
import serial

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CENTER_X, CENTER_Y = 320, 240

KP = 0.25
KI = 0.500
KD = 0.025

RED_LOWER1 = np.array([0, 100, 100])
RED_UPPER1 = np.array([10, 255, 255])
RED_LOWER2 = np.array([160, 100, 100])
RED_UPPER2 = np.array([179, 255, 255])

SEND_INTERVAL = 0.1
SMOOTH_ALPHA = 0.3
MAX_INTEGRAL = 1.0

cap = cv2.VideoCapture(0)
cap.set(3, CAMERA_WIDTH)
cap.set(4, CAMERA_HEIGHT)

arduino = None
try:
    arduino = serial.Serial('/dev/tty.usbmodem12301', 9600, timeout=1)
    time.sleep(2)
    print("[SYSTEM] Arduino connected")
except:
    print("[SYSTEM] Running in simulation mode")

servo_x = 0.0
servo_y = 0.0
integral_x = 0.0
integral_y = 0.0
prev_error_x = 0.0
prev_error_y = 0.0
prev_time = time.time()

last_send_time = 0
last_command = "0.00,0.00\n"
smooth_x = 0.0
smooth_y = 0.0

FONT_PRIMARY = cv2.FONT_HERSHEY_DUPLEX
FONT_SECONDARY = cv2.FONT_HERSHEY_SIMPLEX

FONT_SCALE_XS = 0.35
FONT_SCALE_SM = 0.5
FONT_SCALE_MD = 0.65
FONT_SCALE_LG = 0.8

COLORS = {
    'panel_bg': (30, 35, 45),
    'panel_border': (80, 180, 255),
    'text_primary': (220, 240, 255),
    'text_secondary': (180, 220, 255),
    'text_accent': (255, 220, 100),
    'param_p': (100, 255, 150),
    'param_i': (255, 200, 100),
    'param_d': (100, 200, 255),
    'success': (100, 255, 100),
    'warning': (255, 180, 50),
    'error': (255, 100, 100),
    'target': (0, 200, 255),
    'vector': (100, 255, 150),
    'crosshair': (200, 200, 220),
    'output_x': (100, 220, 255),
    'output_y': (255, 150, 100),
    'error_x': (100, 220, 255),
    'error_y': (255, 150, 100),
    'error_area': (255, 220, 100),
}

def send_servos(x, y):
    global last_send_time, last_command, smooth_x, smooth_y
    if not (arduino and arduino.is_open):
        return False
    current_time = time.time()
    if current_time - last_send_time < SEND_INTERVAL:
        return False
    x = np.clip(float(x), -1.0, 1.0)
    y = np.clip(float(y), -1.0, 1.0)
    smooth_x = smooth_x * (1 - SMOOTH_ALPHA) + x * SMOOTH_ALPHA
    smooth_y = smooth_y * (1 - SMOOTH_ALPHA) + y * SMOOTH_ALPHA
    if abs(smooth_x) < 0.02:
        smooth_x = 0.0
    if abs(smooth_y) < 0.02:
        smooth_y = 0.0
    command = f"{smooth_x:.2f},{smooth_y:.2f}\n"
    if command == last_command:
        return False
    try:
        arduino.write(command.encode())
        arduino.flush()
        last_send_time = current_time
        last_command = command
        return True
    except Exception as e:
        print(f"[ERROR] Serial: {e}")
        return False

def calculate_pid(error_x, error_y, dt):
    global integral_x, integral_y, prev_error_x, prev_error_y
    p_x = KP * error_x
    p_y = KP * error_y
    integral_x += error_x * dt
    integral_y += error_y * dt
    integral_x = np.clip(integral_x, -MAX_INTEGRAL, MAX_INTEGRAL)
    integral_y = np.clip(integral_y, -MAX_INTEGRAL, MAX_INTEGRAL)
    i_x = KI * integral_x
    i_y = KI * integral_y
    d_x = KD * (error_x - prev_error_x) / dt if dt > 0 else 0.0
    d_y = KD * (error_y - prev_error_y) / dt if dt > 0 else 0.0
    prev_error_x = error_x
    prev_error_y = error_y
    output_x = np.clip(p_x + i_x + d_x, -1.0, 1.0)
    output_y = np.clip(p_y + i_y + d_y, -1.0, 1.0)
    return output_x, output_y

def create_mask_visualization(mask):
    mask_viz = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    mask_viz[mask > 0] = [255, 255, 255]
    mask_viz[mask == 0] = [0, 0, 0]
    grid_spacing = 40
    for i in range(0, mask_viz.shape[1], grid_spacing):
        cv2.line(mask_viz, (i, 0), (i, mask_viz.shape[0]), (60, 60, 60), 1)
    for i in range(0, mask_viz.shape[0], grid_spacing):
        cv2.line(mask_viz, (0, i), (mask_viz.shape[1], i), (60, 60, 60), 1)
    title_bg = cv2.rectangle(mask_viz.copy(), (0, 0), (mask_viz.shape[1], 35), (0, 0, 0), -1)
    mask_viz = cv2.addWeighted(title_bg, 0.8, mask_viz, 0.2, 0)
    cv2.putText(mask_viz, "SEGMENTATION MASK", (10, 25), FONT_PRIMARY, 0.7, (220, 220, 240), 2)
    cv2.putText(mask_viz, "White = Detected Area", (10, 50), FONT_SECONDARY, 0.45, (180, 180, 200), 1)
    return mask_viz

def draw_text_with_background(frame, text, position, font, scale, color, thickness=2, bg_color=None):
    x, y = position
    text_size = cv2.getTextSize(text, font, scale, thickness)[0]
    if bg_color is None:
        bg_color = COLORS['panel_bg']
    padding = 5
    cv2.rectangle(
        frame,
        (x - padding, y - text_size[1] - padding),
        (x + text_size[0] + padding, y + padding),
        bg_color,
        -1
    )
    cv2.putText(frame, text, (x, y), font, scale, color, thickness)
    return text_size

def draw_info_panel(frame, ball_detected, pixel_error_x, pixel_error_y, area):
    height, width = frame.shape[:2]

    panel_left = (10, 10, 220, 160)
    cv2.rectangle(frame, (panel_left[0], panel_left[1]), (panel_left[2], panel_left[3]), COLORS['panel_bg'], -1)
    cv2.rectangle(frame, (panel_left[0], panel_left[1]), (panel_left[2], panel_left[3]), COLORS['panel_border'], 2)
    draw_text_with_background(frame, "CONTROL SYSTEM", (25, 40), FONT_PRIMARY, FONT_SCALE_SM, COLORS['text_primary'])
    draw_text_with_background(frame, f"P: {KP:.3f}", (30, 65), FONT_SECONDARY, FONT_SCALE_SM, COLORS['param_p'])
    draw_text_with_background(frame, f"I: {KI:.3f}", (30, 95), FONT_SECONDARY, FONT_SCALE_SM, COLORS['param_i'])
    draw_text_with_background(frame, f"D: {KD:.3f}", (30, 125), FONT_SECONDARY, FONT_SCALE_SM, COLORS['param_d'])
    status_color = COLORS['success'] if ball_detected else COLORS['warning']
    status_text = "TARGET LOCKED" if ball_detected else "NO TARGET"
    draw_text_with_background(frame, status_text, (30, 155), FONT_SECONDARY, FONT_SCALE_SM, status_color)

    panel_right = (width - 230, 10, width - 10, 160)
    cv2.rectangle(frame, (panel_right[0], panel_right[1]), (panel_right[2], panel_right[3]), COLORS['panel_bg'], -1)
    cv2.rectangle(frame, (panel_right[0], panel_right[1]), (panel_right[2], panel_right[3]), COLORS['panel_border'], 2)
    draw_text_with_background(frame, "ERROR METRICS", (panel_right[0] + 15, 40), FONT_PRIMARY, FONT_SCALE_SM, COLORS['text_primary'])

    if ball_detected:
        draw_text_with_background(frame, f"X Error: {pixel_error_x:+4d} px", (panel_right[0] + 20, 65), FONT_SECONDARY, FONT_SCALE_SM, COLORS['error_x'])
        draw_text_with_background(frame, f"Y Error: {pixel_error_y:+4d} px", (panel_right[0] + 20, 95), FONT_SECONDARY, FONT_SCALE_SM, COLORS['error_y'])
        draw_text_with_background(frame, f"Area: {area:.0f} px", (panel_right[0] + 20, 125), FONT_SECONDARY, FONT_SCALE_SM, COLORS['error_area'])
    else:
        draw_text_with_background(frame, "X Error: ---- px", (panel_right[0] + 20, 65), FONT_SECONDARY, FONT_SCALE_SM, COLORS['text_secondary'])
        draw_text_with_background(frame, "Y Error: ---- px", (panel_right[0] + 20, 95), FONT_SECONDARY, FONT_SCALE_SM, COLORS['text_secondary'])
        draw_text_with_background(frame, "Area: ---- px", (panel_right[0] + 20, 125), FONT_SECONDARY, FONT_SCALE_SM, COLORS['text_secondary'])

    panel_bottom = (10, height - 100, 220, height - 10)
    cv2.rectangle(frame, (panel_bottom[0], panel_bottom[1]), (panel_bottom[2], panel_bottom[3]), COLORS['panel_bg'], -1)
    cv2.rectangle(frame, (panel_bottom[0], panel_bottom[1]), (panel_bottom[2], panel_bottom[3]), COLORS['panel_border'], 2)
    draw_text_with_background(frame, "CONTROL OUTPUT", (25, panel_bottom[1] + 25), FONT_PRIMARY, FONT_SCALE_SM, COLORS['text_primary'])
    draw_text_with_background(frame, f"Servo X: {smooth_x:+.2f}", (30, panel_bottom[1] + 55), FONT_SECONDARY, FONT_SCALE_MD, COLORS['output_x'])
    draw_text_with_background(frame, f"Servo Y: {smooth_y:+.2f}", (30, panel_bottom[1] + 85), FONT_SECONDARY, FONT_SCALE_MD, COLORS['output_y'])

    return frame

send_servos(0, 0)
time.sleep(0.5)

print("\n" + "═" * 60)
print("   REAL-TIME VISUAL TRACKING SYSTEM   ")
print("═" * 60)
print("  Controller: PID")
print(f"  Parameters: P={KP:.3f}, I={KI:.3f}, D={KD:.3f}")
print("  Press 'Q' to quit")
print("═" * 60)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.flip(frame, 1)
    height, width = frame.shape[:2]

    current_time = time.time()
    dt = current_time - prev_time
    prev_time = current_time

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, RED_LOWER1, RED_UPPER1)
    mask2 = cv2.inRange(hsv, RED_LOWER2, RED_UPPER2)
    mask = cv2.bitwise_or(mask1, mask2)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=2)

    mask_viz = create_mask_visualization(mask)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    ball_detected = False
    pixel_error_x = 0
    pixel_error_y = 0
    area = 0

    if contours:
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        if area > 300:
            ball_detected = True
            (x, y), radius = cv2.minEnclosingCircle(largest)
            ball_x, ball_y = int(x), int(y)
            norm_error_x = (ball_x - CENTER_X) / CENTER_X
            norm_error_y = (CENTER_Y - ball_y) / CENTER_Y
            servo_x, servo_y = calculate_pid(norm_error_x, norm_error_y, dt)
            send_servos(servo_x, servo_y)

            cv2.circle(frame, (ball_x, ball_y), int(radius), COLORS['target'], 2)
            cv2.circle(frame, (ball_x, ball_y), 7, COLORS['warning'], -1)
            cv2.circle(frame, (ball_x, ball_y), 3, (255, 255, 255), -1)

            arrow_length = min(60, max(30, int(abs(norm_error_x) * 100)))
            arrow_end_x = CENTER_X + int(norm_error_x * arrow_length)
            arrow_end_y = CENTER_Y + int(norm_error_y * arrow_length)
            cv2.arrowedLine(frame, (CENTER_X, CENTER_Y), (arrow_end_x, arrow_end_y), COLORS['vector'], 3, tipLength=0.3, line_type=cv2.LINE_AA)
            cv2.line(frame, (CENTER_X, CENTER_Y), (ball_x, ball_y), (180, 100, 220, 100), 1, lineType=cv2.LINE_AA)

            pixel_error_x = ball_x - CENTER_X
            pixel_error_y = CENTER_Y - ball_y

    if not ball_detected:
        integral_x *= 0.95
        integral_y *= 0.95
        if time.time() - last_send_time > 0.5:
            send_servos(0, 0)
        servo_x = 0.0
        servo_y = 0.0
        draw_text_with_background(frame, "NO TARGET DETECTED", (CENTER_X - 100, CENTER_Y), FONT_PRIMARY, FONT_SCALE_MD, COLORS['warning'])

    cross_size = 25
    cv2.circle(frame, (CENTER_X, CENTER_Y), 10, COLORS['crosshair'], 1, lineType=cv2.LINE_AA)
    cv2.circle(frame, (CENTER_X, CENTER_Y), 3, COLORS['crosshair'], -1)
    cv2.line(frame, (CENTER_X - cross_size, CENTER_Y), (CENTER_X - 5, CENTER_Y), COLORS['crosshair'], 1, lineType=cv2.LINE_AA)
    cv2.line(frame, (CENTER_X + 5, CENTER_Y), (CENTER_X + cross_size, CENTER_Y), COLORS['crosshair'], 1, lineType=cv2.LINE_AA)
    cv2.line(frame, (CENTER_X, CENTER_Y - cross_size), (CENTER_X, CENTER_Y - 5), COLORS['crosshair'], 1, lineType=cv2.LINE_AA)
    cv2.line(frame, (CENTER_X, CENTER_Y + 5), (CENTER_X, CENTER_Y + cross_size), COLORS['crosshair'], 1, lineType=cv2.LINE_AA)

    draw_text_with_background(frame, f"({CENTER_X},{CENTER_Y})", (CENTER_X - 25, CENTER_Y - 25), FONT_SECONDARY, FONT_SCALE_XS, COLORS['text_accent'])

    frame = draw_info_panel(frame, ball_detected, pixel_error_x, pixel_error_y, area)

    cv2.imshow('Real-Time Visual Tracking | PID Control', frame)
    cv2.imshow('Segmentation Mask | White = Detected', mask_viz)

    key = cv2.waitKey(1) & 0xFF
    if key == 27 or key == ord('q'):
        print("\n[SYSTEM] Shutting down...")
        break

cap.release()
cv2.destroyAllWindows()

if arduino:
    send_servos(0, 0)
    time.sleep(0.5)
    arduino.close()

print("\n" + "═" * 60)
print("   SYSTEM SHUTDOWN COMPLETE   ")
print("═" * 60)
print(f"  Final PID Parameters:")
print(f"    P = {KP:.3f}")
print(f"    I = {KI:.3f}")
print(f"    D = {KD:.3f}")
print("═" * 60)
print("  System ready for academic demonstration")
print("═" * 60)

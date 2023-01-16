# 0.initial setting
import time
import RPi.GPIO as GPIO
import pytesseract
import matplotlib.pyplot as plt
import numpy as np
import cv2
import serial

# import for mongodb setting
import src.DBconfig as db
from datetime import datetime

# constants
SERIALPORT = '/dev/ttyUSB0'
SERIALBAUDRATE = 9600


class Car:
    def __init__(self):
        self.parklot_num = 0
        self.numplate = 0
        self.resistor_value = 0
        self.battery_status = 0


# 1.serical connect to arduino


ser = serial.Serial(SERIALPORT, SERIALBAUDRATE, timeout=1)


def serial_wr(send_data_toUno):  # write
    ser.write(send_data_toUno)
    time.sleep(0.5)


def serial_rd():  # read
    try:
        if ser.readable():
            res = ser.readline().strip()
            print(res)
        return (int(res))
    except ValueError as error:
        print(error)


# 2.recognize nunmber plate

def recog_numplate():
    img_ori = cv2.imread('6.jpg')  # ????
    height, width, channel = img_ori.shape
    gray = cv2.cvtColor(img_ori, cv2.COLOR_BGR2GRAY)
    structuringElement = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    imgTopHat = cv2.morphologyEx(gray, cv2.MORPH_TOPHAT, structuringElement)
    imgBlackHat = cv2.morphologyEx(
        gray, cv2.MORPH_BLACKHAT, structuringElement)
    imgGrayscalePlusTopHat = cv2.add(gray, imgTopHat)
    gray = cv2.subtract(imgGrayscalePlusTopHat, imgBlackHat)
    img_blurred = cv2.GaussianBlur(gray, ksize=(5, 5), sigmaX=0)

    img_thresh = cv2.adaptiveThreshold(
        img_blurred,
        maxValue=255.0,
        adaptiveMethod=cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        thresholdType=cv2.THRESH_BINARY_INV,
        blockSize=19,
        C=9
    )

    contours, _ = cv2.findContours(
        img_thresh,
        mode=cv2.RETR_LIST,
        method=cv2.CHAIN_APPROX_SIMPLE
    )

    temp_result = np.zeros((height, width, channel), dtype=np.uint8)
    cv2.drawContours(temp_result, contours=contours,
                     contourIdx=-1, color=(255, 255, 255))
    temp_result = np.zeros((height, width, channel), dtype=np.uint8)
    contours_dict = []

    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(temp_result, pt1=(x, y), pt2=(x+w, y+h),
                      color=(255, 255, 255), thickness=2)
        # insert to dict
        contours_dict.append({
            'contour': contour,
            'x': x,
            'y': y,
            'w': w,
            'h': h,
            'cx': x + (w / 2),
            'cy': y + (h / 2)
        })
        MIN_AREA = 80
    MIN_WIDTH, MIN_HEIGHT = 2, 8
    MIN_RATIO, MAX_RATIO = 0.25, 1.0
    possible_contours = []
    cnt = 0
    for d in contours_dict:
        area = d['w'] * d['h']
        ratio = d['w'] / d['h']
        if area > MIN_AREA \
                and d['w'] > MIN_WIDTH and d['h'] > MIN_HEIGHT \
                and MIN_RATIO < ratio < MAX_RATIO:
            d['idx'] = cnt
            cnt += 1
            possible_contours.append(d)

    # visualize possible contours
    temp_result = np.zeros((height, width, channel), dtype=np.uint8)
    for d in possible_contours:
        # cv2.drawContours(temp_result, d['contour'], -1, (255, 255, 255))
        cv2.rectangle(temp_result, pt1=(d['x'], d['y']), pt2=(
            d['x']+d['w'], d['y']+d['h']), color=(255, 255, 255), thickness=2)
    MAX_DIAG_MULTIPLYER = 5  # 5
    MAX_ANGLE_DIFF = 12.0  # 12.0
    MAX_AREA_DIFF = 0.5  # 0.5
    MAX_WIDTH_DIFF = 0.8
    MAX_HEIGHT_DIFF = 0.2
    MIN_N_MATCHED = 3  # 3

    def find_chars(contour_list):
        matched_result_idx = []

        for d1 in contour_list:
            matched_contours_idx = []
            for d2 in contour_list:
                if d1['idx'] == d2['idx']:
                    continue
                dx = abs(d1['cx'] - d2['cx'])
                dy = abs(d1['cy'] - d2['cy'])
                diagonal_length1 = np.sqrt(d1['w'] ** 2 + d1['h'] ** 2)
                distance = np.linalg.norm(
                    np.array([d1['cx'], d1['cy']]) - np.array([d2['cx'], d2['cy']]))
                if dx == 0:
                    angle_diff = 90
                else:
                    angle_diff = np.degrees(np.arctan(dy / dx))
                area_diff = abs(d1['w'] * d1['h'] - d2['w']
                                * d2['h']) / (d1['w'] * d1['h'])
                width_diff = abs(d1['w'] - d2['w']) / d1['w']
                height_diff = abs(d1['h'] - d2['h']) / d1['h']

                if distance < diagonal_length1 * MAX_DIAG_MULTIPLYER \
                        and angle_diff < MAX_ANGLE_DIFF and area_diff < MAX_AREA_DIFF \
                        and width_diff < MAX_WIDTH_DIFF and height_diff < MAX_HEIGHT_DIFF:
                    matched_contours_idx.append(d2['idx'])

            # append this contour
            matched_contours_idx.append(d1['idx'])
            if len(matched_contours_idx) < MIN_N_MATCHED:
                continue
            matched_result_idx.append(matched_contours_idx)
            unmatched_contour_idx = []
            for d4 in contour_list:
                if d4['idx'] not in matched_contours_idx:
                    unmatched_contour_idx.append(d4['idx'])
            unmatched_contour = np.take(
                possible_contours, unmatched_contour_idx)

            # recursive
            recursive_contour_list = find_chars(unmatched_contour)
            for idx in recursive_contour_list:
                matched_result_idx.append(idx)
            break
        return matched_result_idx

    result_idx = find_chars(possible_contours)
    matched_result = []
    for idx_list in result_idx:
        matched_result.append(np.take(possible_contours, idx_list))
    # visualize possible contours
    temp_result = np.zeros((height, width, channel), dtype=np.uint8)
    for r in matched_result:
        for d in r:
            #         cv2.drawContours(temp_result, d['contour'], -1, (255, 255, 255))
            cv2.rectangle(temp_result, pt1=(d['x'], d['y']), pt2=(
                d['x']+d['w'], d['y']+d['h']), color=(255, 255, 255), thickness=2)
    PLATE_WIDTH_PADDING = 1.3  # 1.3
    PLATE_HEIGHT_PADDING = 1.5  # 1.5
    MIN_PLATE_RATIO = 3
    MAX_PLATE_RATIO = 10
    plate_imgs = []
    plate_infos = []

    for i, matched_chars in enumerate(matched_result):
        sorted_chars = sorted(matched_chars, key=lambda x: x['cx'])
        plate_cx = (sorted_chars[0]['cx'] + sorted_chars[-1]['cx']) / 2
        plate_cy = (sorted_chars[0]['cy'] + sorted_chars[-1]['cy']) / 2
        plate_width = (sorted_chars[-1]['x'] + sorted_chars[-1]
                       ['w'] - sorted_chars[0]['x']) * PLATE_WIDTH_PADDING
        sum_height = 0
        for d in sorted_chars:
            sum_height += d['h']
        plate_height = int(sum_height / len(sorted_chars)
                           * PLATE_HEIGHT_PADDING)
        triangle_height = sorted_chars[-1]['cy'] - sorted_chars[0]['cy']
        triangle_hypotenus = np.linalg.norm(
            np.array([sorted_chars[0]['cx'], sorted_chars[0]['cy']]) -
            np.array([sorted_chars[-1]['cx'], sorted_chars[-1]['cy']])
        )

        angle = np.degrees(np.arcsin(triangle_height / triangle_hypotenus))
        rotation_matrix = cv2.getRotationMatrix2D(
            center=(plate_cx, plate_cy), angle=angle, scale=1.0)
        img_rotated = cv2.warpAffine(
            img_thresh, M=rotation_matrix, dsize=(width, height))
        img_cropped = cv2.getRectSubPix(
            img_rotated,
            patchSize=(int(plate_width), int(plate_height)),
            center=(int(plate_cx), int(plate_cy))
        )
        if img_cropped.shape[1] / img_cropped.shape[0] < MIN_PLATE_RATIO or img_cropped.shape[1] / img_cropped.shape[0] < MIN_PLATE_RATIO > MAX_PLATE_RATIO:
            continue
        plate_imgs.append(img_cropped)
        plate_infos.append({
            'x': int(plate_cx - plate_width / 2),
            'y': int(plate_cy - plate_height / 2),
            'w': int(plate_width),
            'h': int(plate_height)
        })

        longest_idx, longest_text = -1, 0
    plate_chars = []

    for i, plate_img in enumerate(plate_imgs):
        plate_img = cv2.resize(plate_img, dsize=(0, 0), fx=1.6, fy=1.6)
        _, plate_img = cv2.threshold(
            plate_img, thresh=0.0, maxval=255.0, type=cv2.THRESH_BINARY | cv2.THRESH_OTSU)
        # find contours again (same as above)
        contours, _ = cv2.findContours(
            plate_img, mode=cv2.RETR_LIST, method=cv2.CHAIN_APPROX_SIMPLE)
        plate_min_x, plate_min_y = plate_img.shape[1], plate_img.shape[0]
        plate_max_x, plate_max_y = 0, 0

        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            area = w * h
            ratio = w / h
            if area > MIN_AREA \
                    and w > MIN_WIDTH and h > MIN_HEIGHT \
                    and MIN_RATIO < ratio < MAX_RATIO:
                if x < plate_min_x:
                    plate_min_x = x
                if y < plate_min_y:
                    plate_min_y = y
                if x + w > plate_max_x:
                    plate_max_x = x + w
                if y + h > plate_max_y:
                    plate_max_y = y + h

        img_result = plate_img[plate_min_y:plate_max_y,
                               plate_min_x:plate_max_x]
        img_result = cv2.GaussianBlur(img_result, ksize=(3, 3), sigmaX=0)
        _, img_result = cv2.threshold(
            img_result, thresh=0.0, maxval=255.0, type=cv2.THRESH_BINARY | cv2.THRESH_OTSU)
        img_result = cv2.copyMakeBorder(
            img_result, top=10, bottom=10, left=10, right=10, borderType=cv2.BORDER_CONSTANT, value=(0, 0, 0))
        chars = pytesseract.image_to_string(
            img_result, lang='kor', config='--psm 7 --oem 0')
        result_chars = ''
        has_digit = False
        for c in chars:
            if ord('가') <= ord(c) <= ord('힣') or c.isdigit():
                if c.isdigit():
                    has_digit = True
                result_chars += c
        return result_chars
# 3.Check battery status


def check_btstatus():
    pass

# 4.Toggle LED


def toggle_led(battery_status):
    # initial setting
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(17, GPIO.OUT)

    if (battery_status < 500):
        for i in range(2):
            GPIO.output(17, True)
            time.sleep(2)
            GPIO.output(17, False)


# 5.Send data to DB
def initializeParkingStatus(collection):

    print("\n>> Initializing database")
    # Values to be updated.
    info = {'carnum': "",
            'battery': 0,
            'carstatus': "",
            'checktime': datetime.now()}
    collection.update_many({'occupied': True}, {"$set": info})
    # Get Data from DB
    for item in collection.find({}):
        print(item, end="\n")
    return


def updateParkingStatus(data, collection):

    # DB location
    location = {'location': data.parklot_num}
    # Values to be updated.
    info = {'carnum': data.numplate,
            'battery': data.battery_status,
            'carstatus': bool(data.resistor_value),
            'occupied': True,
            'checktime': datetime.now()}
    print(f'{info}\n')
    # update DB
    collection.update_one(location, {"$set": info})
    print("\n>> Database updated")
    # Get Data from DB
    for item in collection.find(location):
        print(item, end="\n")

    return

# 6.main


def main():
    test_target = Car()

    # Get database info
    collection = db.get_database()
    # Initialize DB
    initializeParkingStatus(collection)

    test_target = Car()
    while (1):
        try:
            from_ino = serial_rd()
            if (from_ino != 0):
                test_target.resistor_value = (
                    from_ino - (from_ino//10000)*10000)//1000  # from_ino
                test_target.parklot_num = from_ino//10000  # from_ino
                test_target.numplate = recog_numplate()
                test_target.battery_status = (from_ino % 1000)/10
                # 4. excute toggle LED
                toggle_led(test_target.battery_status)
                # 5. execute DB update
                updateParkingStatus(test_target, collection)

                # 6. initialize received data
                from_ino = 0
                # time.sleep(1000)
                # serial_wr(b'1\n')
        except TypeError as error:
            print(error)


if __name__ == "__main__":
    main()

'''
Need to fix
1. recog numplate method's input 
2. data length rcv from ino
3. serial rd
4. checking battery status method 
5. LED toggling pin
6. exact DB words
'''

# # test data
# testdata = {
#     "parklot_num": 1,  # 차고번호 1 to 3
#     "resistor_value": 1,  # 가변저항값 => 0이면 고장(false), 1이면 정상(true)
#     "numplate": "마바다 대구 1234",  # 번호판 인식값
#     "battery_status": 50
# }
# # set test data to car class
# test_target.parklot_num = testdata["parklot_num"]
# test_target.resistor_value = testdata["resistor_value"]
# test_target.numplate = testdata["numplate"]
# test_target.battery_status = testdata["battery_status"]

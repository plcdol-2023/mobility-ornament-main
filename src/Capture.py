import cv2


def Load_Camera(index: int):

    
    print(">> 카메라 로드 중...")

    # VideoCapture : 카메라 열기
    capture = cv2.VideoCapture(index)
    print(type(capture))
    print(capture)

    # 원본 동영상 크기 정보
    w = capture.get(cv2.CAP_PROP_FRAME_WIDTH)
    h = capture.get(cv2.CAP_PROP_FRAME_HEIGHT)
    print(">> 원본 동영상 너비(가로) : {}, 높이(세로) : {}".format(w, h))

    # 동영상 크기 변환
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)  # 가로
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)  # 세로

    # 변환된 동영상 크기 정보
    w = capture.get(cv2.CAP_PROP_FRAME_WIDTH)
    h = capture.get(cv2.CAP_PROP_FRAME_HEIGHT)

    print(">> 변환된 동영상 너비(가로) : {}, 높이(세로) : {}".format(w, h))
    print(">> 카메라 로드 완료.")

    while True:
        retval, FRAME = capture.read()
        # 읽은 프레임이 없는 경우 종료
        if not retval:
            break

    # 카메라 메모리 연결 해제
    capture.release()
    print(">> release memory")
    return 

if (__name__ == "__main__"):
    Load_Camera(0)

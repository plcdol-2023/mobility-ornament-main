# 필요한 패키지 import
import cv2 # OpenCV(실시간 이미지 프로세싱) 모듈

# 카메라 index 번호
camera = 0

# VideoCapture : 카메라 열기
capture = cv2.VideoCapture(camera)

# 원본 동영상 크기 정보
w = capture.get(cv2.CAP_PROP_FRAME_WIDTH)
h = capture.get(cv2.CAP_PROP_FRAME_HEIGHT)
print("원본 동영상 너비(가로) : {}, 높이(세로) : {}".format(w, h))

# 동영상 크기 변환
capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1920) # 가로
capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080) # 세로

# 변환된 동영상 크기 정보
w = capture.get(cv2.CAP_PROP_FRAME_WIDTH)
h = capture.get(cv2.CAP_PROP_FRAME_HEIGHT)
print("변환된 동영상 너비(가로) : {}, 높이(세로) : {}".format(w, h))

from threading import Thread

def _listener_():
    while True: 
        doCapture=input("캡쳐?")

        if(doCapture=='y'):
            print("캡쳐")
            cv2.imwrite('images/img_captured_test.jpg', frame, params=[cv2.IMWRITE_JPEG_QUALITY,100])
        
Listener = Thread(name="_listener_", target=_listener_,
                          args=(), daemon=True)
Listener.start()

w_min =int( w*0.25 )
w_max =int( w*0.75 )
h_min =int( h*0.07 )
h_max =int( h*0.93 )

print(f'{w_min} X {w_max} | {h_min} X {h_max} ')
while True:
    # read : 프레임 읽기
    # [return]
    # 1) 읽은 결과(True / False)
    # 2) 읽은 프레임
    retval, frame = capture.read()
    
    frame=frame[h_min:h_max,int(w_min):int(w_max)]
    print(frame)
    # 읽은 프레임이 없는 경우 종료
    if not retval:
        break
   
    
    # 프레임 출력
    cv2.imshow("resize_frame", frame)
    
    # 'q' 를 입력하면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 동영상 파일 또는 카메라를 닫고 메모리를 해제
capture.release()

# 모든 창 닫기
cv2.destroyAllWindows()

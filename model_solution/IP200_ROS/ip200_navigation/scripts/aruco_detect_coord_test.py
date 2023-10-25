# ROS 노드가 아니라 카메라를 연결하고 카메라로 aruco마커 detect 테스트하는 코드입니다.
 
# Requirements
import cv2
import numpy as np

# Parameters
aruco_type = cv2.aruco.DICT_6X6_250
cam_src_num = 4
show_img = True

# Calibration Data
# intrinsic_camera = np.array(((594.0705387, 0., 310.21281441), (0., 591.79870457, 250.43981239), (0., 0., 1.)))
# distortion = np.array([[ 2.67285112e-01, -1.18930743e+00, -9.60580634e-04,  6.35610786e-03, 1.16625781e+00]])
intrinsic_camera = np.array(((616.1589965820312, 0.0, 336.71173095703125), (0.0, 616.2099609375, 232.3656463623047), (0.0, 0.0, 1.0)))
distortion = np.array([[0.0, 0.0, 0.0, 0.0, 0.0]])

# Estimation Function
def pose_estimation(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters()
    # detector = cv2.aruco.ArucoDetector(cv2.aruco_dict, parameters)
    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict,parameters=parameters)
    # corners, ids, rejected_img_points = detector.detectMarkers(gray)#, cv2.aruco_dict,parameters=parameters)

    if len(corners) > 0:
        for i in range(0, len(ids)):
           
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.024, matrix_coefficients, distortion_coefficients)
            # print(f"rvec: {rvec}")
            
            
            xax, yax, zax = rvec[0][0]  # degree of rotation
            xax, yax, zax = map(int, (xax*57.2958, yax*57.2958, zax*57.2958))
            distance = np.linalg.norm(tvec)

            # # 마커의 Z축 방향에서 1m 떨어진 지점까지의 거리 계산
            # distance_to_target = tvec[0][0][2] - 1.0
            
            # # 벡터 A와 B 정의
            # target_direction = tvec[0][0] + np.array([0, 0, 1.0])

            # camera_direction = np.array([0, 0, 1])
            # dot_product = np.dot(target_direction, camera_direction)
            # magnitude_target = np.linalg.norm(target_direction)
            # magnitude_camera = np.linalg.norm(camera_direction)

            # angle_between = np.arccos(dot_product / (magnitude_target * magnitude_camera))
            # angle_between_deg = np.degrees(angle_between)
            angle_xz = np.degrees(np.arctan2(tvec[0][0][2], tvec[0][0][0]))
            print(f"Angle between x and z: {angle_xz:.2f} degrees")
            print(tvec)

            # 1. rvec을 3x3 회전 행렬로 변환합니다.
            R, _ = cv2.Rodrigues(rvec[0][0])

            # 2. 변환된 회전 행렬을 사용하여 카메라 좌표계의 Z축이 마커 좌표계에서 어떻게 보이는지 계산합니다.
            transformed_z = np.dot(R, np.array([0, 0, 1]))

            # 3. 마커의 Z축과 변환된 Z축 간의 각도를 계산합니다.
            cos_angle = np.dot(transformed_z, np.array([0, 0, 1]))
            angle_between_z_axes = np.degrees(np.arccos(np.clip(cos_angle, -1.0, 1.0)))

            print(f"Angle between marker's Z axis and camera's Z axis: {angle_between_z_axes:.2f} degrees")
            # print(f"Distance to marker: {distance:.2f} meters")
            # print(f"Distance to target point: {distance_to_target:.2f} meters")
            # print(f"Angle between distance_to_target and distance: {angle_between_deg:.2f} degrees")
            
            # print(f"Distance from the marker: {distance} meters")

            if show_img:
                cv2.putText(frame, str((xax, yax, zax)), (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(frame, str(f"distance: {distance}"), (50, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.05)
                cv2.aruco.drawDetectedMarkers(frame, corners)

            else:
                print("X:", xax, "Y:", yax, "Z:", zax)

    return frame

# Visualize and Print XYZ Degrees
cap = cv2.VideoCapture(cam_src_num)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 512)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 512)

while cap.isOpened():
    ret, img = cap.read()
    output = pose_estimation(img, aruco_type, intrinsic_camera, distortion)

    if show_img:
        cv2.imshow('Estimated Pose', output)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    
cap.release()
cv2.destroyAllWindows()
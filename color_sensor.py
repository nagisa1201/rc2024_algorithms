import cv2 
import numpy as np
import threading
import pyrealsense2 as rs
import rospy
import time
from std_msgs.msg import Int8
video_save_interval = 30  # 保存间隔，单位为秒
last_save_time = time.time()  # 上次保存时间的初始值为当前时间
out_path = '/home/nvidia/video_RC/color/'
fourcc = cv2.VideoWriter_fourcc(*'mp4v')  
filename = f'{out_path}output_{0}.mp4'
out = cv2.VideoWriter(filename, fourcc, 30.0, (640, 480))

class Trace_Colors:
    def __init__(self, image, lparam=None, hparam=None):
        self.image =image
        self.lparam = lparam
        self.hparam = hparam

        cv2.namedWindow('Tracking')
        cv2.resizeWindow('Tracking', 800, 600)
        cv2.createTrackbar('LH', 'Tracking', 0, 255, self.nothing)
        cv2.createTrackbar('UH', 'Tracking', 255, 255, self.nothing)
        cv2.createTrackbar('LS', 'Tracking', 0, 255, self.nothing)
        cv2.createTrackbar('US', 'Tracking', 255, 255, self.nothing)
        cv2.createTrackbar('LV', 'Tracking', 0, 255, self.nothing)
        cv2.createTrackbar('UV', 'Tracking', 255, 255, self.nothing)
    def nothing(self, x):
        pass
        
    # 当用find_threshold调整好阈值，输入阈值后使用
    def find(self):
        hsv_img = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)   # 转换位HSV格式
        mask = cv2.inRange(hsv_img, self.lparam, self.hparam)
        res = cv2.bitwise_and(self.image, self.image, mask=mask)
        return res
 
    # 调整阈值
    def choice_thresholdThread(self):
         
        self.lh = cv2.getTrackbarPos('LH', 'Tracking')
        self.uh = cv2.getTrackbarPos('UH', 'Tracking')
        self.ls = cv2.getTrackbarPos('LS', 'Tracking')
        self.us = cv2.getTrackbarPos('US', 'Tracking')
        self.lv = cv2.getTrackbarPos('LV', 'Tracking')
        self.uv = cv2.getTrackbarPos('UV', 'Tracking')

    def update(val):
        pass
    cv2.namedWindow('Adjustments')
    cv2.createTrackbar('Contrast', 'Adjustments', 10, 30, update)  # 初始值10，范围0-30
    cv2.createTrackbar('Brightness', 'Adjustments', 50, 100, update)  # 初始值50，范围0-100

    def reduce_vignetting(image):
    # 通过简单的算术方法减轻暗角效果
        # rows, cols = image.shape[:2]

        # # 创建径向渐变掩码
        # X_resultant_kernel = cv2.getGaussianKernel(cols, cols / 2)
        # Y_resultant_kernel = cv2.getGaussianKernel(rows, rows / 2)
        # resultant_kernel = Y_resultant_kernel * X_resultant_kernel.T
        # mask = 255 * resultant_kernel / np.linalg.norm(resultant_kernel)
        # mask = cv2.normalize(mask, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        # mask = cv2.merge([mask, mask, mask])  # 将单通道掩码转换为三通道

        # # 应用掩码来减轻暗角效果
        # result = cv2.multiply(image, mask, scale=1/255)
        rows, cols = image.shape[:2]

    # 创建径向渐变掩码
        X_resultant_kernel = cv2.getGaussianKernel(cols, cols / 2)
        Y_resultant_kernel = cv2.getGaussianKernel(rows, rows / 2)
        resultant_kernel = Y_resultant_kernel * X_resultant_kernel.T
        mask = 255 * resultant_kernel / np.linalg.norm(resultant_kernel)
        mask = cv2.normalize(mask, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        mask = cv2.merge([mask, mask, mask])  # 将单通道掩码转换为三通道

        # 反转掩码使中间略微变亮，四周变暗
        mask = 255 - mask

        # 应用掩码来减轻暗角效果
        result = cv2.addWeighted(image, 0.8, mask, 0.2, 0)

        return result
     
    def find_threshold(self):
        lower_param=np.array([self.lh,self.ls,self.lv])
        upper_param=np.array([self.uh,self.us,self.uv])
        hsv_img = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)  # 转换位HSV格式
 
        mask = cv2.inRange(hsv_img, lower_param, upper_param) # 创建mask   
        self.res = cv2.bitwise_and(self.image, self.image, mask=mask)        
        return self.res
    def find_threshold_once(self):
        
        # 创建轨迹栏
        cv2.namedWindow('Tracking2')
        cv2.resizeWindow('racking2', 800, 600)  
        cv2.createTrackbar('LH', 'Tracking', 0, 255, self.nothing)
        cv2.createTrackbar('UH', 'Tracking', 255, 255, self.nothing)
        cv2.createTrackbar('LS', 'Tracking', 0, 255, self.nothing)
        cv2.createTrackbar('US', 'Tracking', 255, 255, self.nothing)
        cv2.createTrackbar('LV', 'Tracking', 0, 255, self.nothing)
        cv2.createTrackbar('UV', 'Tracking', 255, 255, self.nothing)
        while True:
            # 轨迹栏参数定义
            lh = cv2.getTrackbarPos('LH', 'Tracking')
            uh = cv2.getTrackbarPos('UH', 'Tracking')
            ls = cv2.getTrackbarPos('LS', 'Tracking')
            us = cv2.getTrackbarPos('US', 'Tracking')
            lv = cv2.getTrackbarPos('LV', 'Tracking')
            uv = cv2.getTrackbarPos('UV', 'Tracking')
 
            lower_param = np.array([lh, ls, lv])
            upper_param = np.array([uh, us, uv])
            hsv_img = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)  # 转换位HSV格式
 
            mask = cv2.inRange(hsv_img, lower_param, upper_param) # 创建mask   
            res = cv2.bitwise_and(self.image, self.image, mask=mask)
 
            cv2.imshow('res', res)
            if cv2.waitKey(1) == 27 or cv2.waitKey(1) == ord('q'):
                cv2.destroyAllWindows()
                break

def overexpose_image(image,gamma):
    results=np.uint8(cv2.pow(image/255.0,gamma)*255.0)
    return results
class Trace_Colors:
    def __init__(self):
        self.lower_red = np.array([153, 86, 69])    # 红色的低阈值
        self.upper_red = np.array([179, 255, 255])  # 红色的高阈值
        self.lower_blue = np.array([70, 61, 40])     # 蓝色的低阈值
        self.upper_blue = np.array([111, 255, 255]) # 蓝色的高阈值
        self.lower_purple = np.array([129, 50, 50]) # 紫色的低阈值
        self.upper_purple = np.array([140, 255, 255]) # 紫色的高阈值

    def find_color_pixels(self, image, color_name):
        hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        if color_name == 'red':
            mask = cv2.inRange(hsv_img, self.lower_red, self.upper_red)
        elif color_name == 'blue':
            mask = cv2.inRange(hsv_img, self.lower_blue, self.upper_blue)
        elif color_name == 'purple':
            mask = cv2.inRange(hsv_img, self.lower_purple, self.upper_purple)
        else:
            return 0
        res = cv2.bitwise_and(image, image, mask=mask)
        color_count = np.count_nonzero(mask)
        return color_count,res

def get_aligned_images(pipeline, align, profile):   
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    aligned_depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()

    intr = color_frame.profile.as_video_stream_profile().intrinsics
    color_sensor = profile.get_device().query_sensors()[1]

    depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics

    camera_parameters = {'fx': intr.fx, 'fy': intr.fy,
                         'ppx': intr.ppx, 'ppy': intr.ppy,
                         'height': intr.height, 'width': intr.width,
                         'depth_scale': profile.get_device().first_depth_sensor().get_depth_scale()
                         }

    depth_image = np.asanyarray(aligned_depth_frame.get_data())
    depth_image_8bit = cv2.convertScaleAbs(depth_image, alpha=0.03)
    depth_image_3d = np.dstack((depth_image_8bit, depth_image_8bit, depth_image_8bit))
    color_image = np.asanyarray(color_frame.get_data())
    return intr, depth_intrin, color_image, depth_image, aligned_depth_frame
def overexpose_image(image,gamma):
    results=np.uint8(cv2.pow(image/255.0,gamma)*255.0)
    return results
def main():
    global last_save_time,out
    lock=0
    rospy.init_node("colorlog_node", anonymous=True)
    pub = rospy.Publisher("colorlog",Int8, queue_size=10)
    rospy.loginfo("6/12第一版追踪")
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device("233622079143") 
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    profile = pipeline.start(config)
    align = rs.align(rs.stream.color)
    color_sensor = profile.get_device().query_sensors()[1]
    color_sensor.set_option(rs.option.enable_auto_exposure, 0)  # 禁用自动曝光
    # exposure_time = 1 # 设置曝光时间为10000微秒，即10毫秒
    # color_sensor.set_option(rs.option.exposure, exposure_time)
    exposure_value = 200  # 根据需要调整
    color_sensor.set_option(rs.option.exposure, exposure_value)
    color_sensor.set_option(rs.option.enable_auto_white_balance, 0)  # 禁用自动白平衡
    trace_colors = Trace_Colors()
    y1,y2,x1,x2=int(480*0.677),480,0,640
    edge = 25000  # Set the edge value for pixel count
    rate = rospy.Rate(40)
    while not rospy.is_shutdown():
        start = time.time()
        intr, depth_intrin, color_image, depth_image, aligned_depth_frame = get_aligned_images(pipeline, align, profile)
        

        color_image=overexpose_image(color_image,1.7)

        roi = color_image[y1:y2, x1:x2]
        # roi=color_image[y1:y2, x1:x2]
        # Detecting and counting red, blue, purple pixels
        red_count,roi_red = trace_colors.find_color_pixels(roi, 'red')
        blue_count,roi_blue = trace_colors.find_color_pixels(roi, 'blue')
        purple_count,roi_purple = trace_colors.find_color_pixels(roi, 'purple')

        # Determine the most dominant color
        max_count = max(red_count, blue_count, purple_count)
        if max_count <= 150:
            dominant_color = 'unknown'
        elif max_count == red_count:
            dominant_color = 'red'
        elif max_count == blue_count:
            dominant_color = 'blue'
        else:
            dominant_color = 'purple'

        # 标签列表
        tags = ['Red', 'Blue', 'Purple', 'Original']

        # 在每个图像的左上角添加标签
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 1
        color = (255, 255, 255)  # 白色
        thickness = 2

        cv2.putText(roi_red, tags[0], (10, 30), font, font_scale, color, thickness)
        cv2.putText(roi_blue, tags[1], (10, 30), font, font_scale, color, thickness)
        cv2.putText(roi_purple, tags[2], (10, 30), font, font_scale, color, thickness)
        cv2.putText(roi, tags[3], (10, 30), font, font_scale, color, thickness)


        # 将图像水平组合成两行
        top_row = np.hstack((roi_red, roi_blue))
        bottom_row = np.hstack((roi_purple, roi))

        # 将两行垂直组合成一个2x2的大图
        combined_image = np.vstack((top_row, bottom_row))
        # Publish the color flag if counts exceed the edge value
        if(lock==1):     
                if red_count < edge and blue_count < edge and purple_count < edge:
                    lock=0
        if(lock==0):     
                if red_count > edge:
                    pub.publish(1)
                    lock=1
                elif blue_count > edge:
                    pub.publish(2)
                    lock=1
                elif purple_count > edge:
                    lock=1
        #当红蓝紫任意一个像素值触发上升沿时触发发布对应数值和锁定，所有像素值均小于（下降沿）触发解锁
        end = time.time()
        fps = 1 / (end - start)

        # Display the image with detected color counts
        cv2.putText(color_image, f"red: {red_count}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(color_image, f"blue: {blue_count}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        cv2.putText(color_image, f"purple: {purple_count}", (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, (128, 0, 128), 2)
        cv2.putText(color_image, f"most: {dominant_color}", (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(color_image, f'FPS: {int(fps)}', (10, 190), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

        cv2.imshow("Color Detection", color_image)
        cv2.imshow('combined', combined_image)
        #保存视频
        current_time = time.time()
        elapsed_time = current_time - last_save_time
        if elapsed_time >= video_save_interval:

            # 如果out尚未初始化，则初始化VideoWriter
            out.release()
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            filename = f'{out_path}output_{current_time}.mp4'
            out = cv2.VideoWriter(filename, fourcc, 30.0, (640, 480))
            last_save_time = current_time
            # 写入当前帧到视频
        out.write(combined_image)
        rate.sleep()
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    pipeline.stop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
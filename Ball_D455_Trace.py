import cv2
from sensor_msgs.msg import Image
import rospy
from ultralytics import YOLO
import numpy as np
import threading
import numpy as np
import pyrealsense2 as rs
from yolov8_ros.msg import R2
from std_msgs.msg import Int32
from cv_bridge import CvBridge
from filterpy.kalman import KalmanFilter
from typing import List
import time
from collections import defaultdict
track_history = defaultdict(lambda: [])
track_ball_id = 0
#5分钟判断一次阵营
judge_camp_flag=False
video_save_interval = 30  # 保存间隔，单位为秒
last_save_time = time.time()  # 上次保存时间的初始值为当前时间
out_path = '/home/nvidia/video_RC/ball_trace/'
fourcc = cv2.VideoWriter_fourcc(*'mp4v')  
filename = f'{out_path}output_{0}.mp4'
out = cv2.VideoWriter(filename, fourcc, 30.0, (640, 480))

class Ball:

    def __init__(self,color,xx,yy,x1,x2,y1,y2,id,depth_intrin):
        #红0，蓝1，紫2
        self.color=color;self.xx=int(xx);self.yy=int(yy);self.x1=int(x1);self.x2=int(x2)
        self.y1=int(y1);self.y2=int(y2);self.id=id
        if xx >=30 and xx<=605: 
            self.distance = aligned_depth_frame.get_distance(xx, yy)
            self.camera_xyz=rs.rs2_deproject_pixel_to_point(depth_intrin, [self.xx,self.yy],self.distance)
            self.camera_xyz=np.round(np.array(self.camera_xyz),3)
            self.base_xyz=coordinate_tf(self.camera_xyz)
#球的定义
    color=0;xx=0;yy=0;x1=0;y1=0;y2=0;id=0
#距离相机初始化为极大值
    distance=99
#在相机坐标系中的xyz
    camera_xyz=[]
#在车头坐标系中的xyz
    base_xyz=[]
#异常情况
    abnormal=False
#判断阵营
def judge_camp(Ball_List:List[Ball]):
    red_num=0
    blue_num=0
    for Ball in Ball_List:
        if Ball.color==0.0:
            red_num+=1
        elif Ball.color==1.0:
            blue_num+=1
    #红方为0
    if red_num>2:
        return 0
    elif blue_num>2:
    #蓝方为1
        return 1
    return None
def decide(Ball_List:List[Ball]):
    if(len(Ball_List)==0):
        return None
    else:
        #按照距离升序
        #Ball_List.sort(key=lambda k:k.distance)原来代码
        Ball_List.sort(key=lambda k:k.distance)
        for i in range(len(Ball_List)):
            #如果是红或者蓝
            if((Ball_List[i].color==1.0 or Ball_List[i].color==0.0) and Ball_List[i].distance<50 ):
                return Ball_List[i]
    return None
#传入参数为球的列表，和追踪的球表示更新追踪的球
#传入的参数为球的列表，不传入追踪的球表示跟踪之前的球
#传出跟新的球如果跟丢了返回空
def Trace(Ball_List:List[Ball],frame,Ball:Ball=None):
    #如果传入球，表示新的的球
    global track_ball_id
    if(Ball!=None):
        track_ball_id=Ball.id
    
    #找到选择的更新的球
    track_ball=None
    for Ball in Ball_List:
        if Ball.id==track_ball_id:
            track_ball=Ball
    #如果跟丢了(找不到)
    if track_ball==None:
        return None
    #如果跟踪的球在边缘或者颜色不对或者被包围着放弃追踪
    if track_ball.color==2.0 or track_ball.distance>50 or track_ball.abnormal:
        return None
    track=track_history[track_ball.id]
    track.append((float(track_ball.xx), float(track_ball.yy)))
    if len(track) > 30:
        track.pop(0)
    points = np.hstack(track).astype(np.int32).reshape((-1, 1, 2))
    cv2.polylines(frame, [points], isClosed=False, color=(230, 230, 230), thickness=10)
    return track_ball   
#把xyz周围的R_min到R_max之间标记出来  //比较难,放弃
def mark_zone(frame,mark_xyz,R_min,R_max):
    return None

#判断方向
def get_direction(track_xyz, ball_xyz):
    dx = ball_xyz[0] - track_xyz[0]
    dy = ball_xyz[1] - track_xyz[1]
    
    angle = np.arctan2(dy, dx) * 180 / np.pi
    d_angle=50
    if 90-d_angle <= angle <=90+d_angle:
        return 'sp'
    return ''

#判断周围有没有球,传入当前帧，球的列表，追踪球中心点，球的半径
def Abnormal_judgment(Ball_List:List[Ball],frame,trace_ball:Ball,R_min,R_max):
    track_xyz=trace_ball.base_xyz
    #找到周围的球
    around_ball=[]
    for Ball in Ball_List:
        if len(Ball.base_xyz)!=0 and (R_min<np.sqrt((Ball.base_xyz[0]-track_xyz[0])**2+(Ball.base_xyz[1]-track_xyz[1])**2)<R_max):
            around_ball.append(Ball)
    #在球框的左上角做个标记
    sp_count=0
    for Ball in around_ball:
        direction = get_direction(track_xyz, Ball.base_xyz)
        cv2.putText(frame, f'Ab{direction}', (Ball.x1,Ball.y1-20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, 1)
        if(direction=='sp' and Ball.color==2.0):
            sp_count+=1
    if(sp_count>=2):
        trace_ball.abnormal=True
        return True

    return False
        
# # 定义状态转移矩阵 A，观测矩阵 H，过程噪声协方差 Q，测量噪声协方差 R，初始状态协方差 P
kf = KalmanFilter(dim_x=1, dim_z=1)
kf2 = KalmanFilter(dim_x=1, dim_z=1)
kf.F = np.array([[1]])    # State transition matrix
kf.H = np.array([[1]])    # Measurement function
kf.Q = np.array([[1]])    # Process uncertainty
kf.R = np.array([[10]])   # Measurement uncertainty
kf.x = np.array([[0]])    # Initial state estimate
kf.P = np.array([[1]])    # Initial covariance estimate
kf2.F = np.array([[1]])    # State transition matrix
kf2.H = np.array([[1]])    # Measurement function
kf2.Q = np.array([[1]])    # Process uncertainty
kf2.R = np.array([[10]])   # Measurement uncertainty
kf2.x = np.array([[0]])    # Initial state estimate
kf2.P = np.array([[1]])    # Initial covariance estimate 原卡尔曼滤波

model = YOLO('/home/nvidia/R2_ws/src/yolov8_ros/scripts/ballbest.engine')
#model = YOLO('/home/nvidia/ballbest.pt')
bridge = CvBridge()

def coordinate_tf(camera_xyz):
    theta_x = 65 / 180 * 3.1415926535
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(theta_x), -np.sin(theta_x)],
        [0, np.sin(theta_x), np.cos(theta_x)]
    ])
    T = np.array([-1.0, 0.0, 0.0])
    point_base = np.dot(Rx, camera_xyz)
    rospy.loginfo(point_base)
    return point_base

def get_aligned_images(pipeline, align):
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    aligned_depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()

    intr = color_frame.profile.as_video_stream_profile().intrinsics
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

def timer_callback():
    global judge_camp_flag
    judge_camp_flag=False
    # print("\n\n\n\n\\n\n\n\n\n\n\n")
    


if __name__ == "__main__":
   
    rospy.init_node("Ball_D435i", anonymous=True)
   
    pub = rospy.Publisher("Chassis_xy", R2, queue_size=10)
    pub2=rospy.Publisher("camp",Int32,queue_size=1)
    rospy.loginfo("6/12第一版追踪")
    r2 = R2()
    camp=Int32()
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device("135222251878") 
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    profile = pipeline.start(config)
    align = rs.align(rs.stream.color)
    rate = rospy.Rate(40)
    start_time=0
    print("加载完毕")
    timer=threading.Timer(180,timer_callback).start()
    try:
        while not rospy.is_shutdown():
            intr, depth_intrin, color_image, depth_image, aligned_depth_frame = get_aligned_images(pipeline, align)
            if not depth_image.any() or not color_image.any():
                continue

            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            images = np.hstack((color_image, depth_colormap))

            results = model.track(color_image, conf=0.65, iou=0.2,persist=True)
            frame = results[0].plot()
            fps = 1000.0 / results[0].speed['inference']
            detach_list=[]
            for r in results[0].boxes :
                if r.id==None:
                    continue
                x1 = r.xyxy[0][0].item()
                y1 = r.xyxy[0][1].item()
                x2 = r.xyxy[0][2].item()
                y2 = r.xyxy[0][3].item()
                xx = np.int64((x1 + x2) / 2)
                yy = np.int64((y1 + y2) / 2)
                #将检测到的球都丢进列表
                detach_list.append(Ball(r.cls.item(),xx,yy,x1,x2,y1,y2,r.id.int().item(),depth_intrin=depth_intrin))
            #追踪
            Trace_ball=Trace(detach_list,frame)
            #判断阵营
            if(judge_camp_flag==False):
                camp.data=judge_camp(detach_list)
                if camp.data!=None:
                    pub2.publish(camp)
                    judge_camp_flag=True
            #延500ms再次追踪
            if(Trace_ball==None and (time.perf_counter()-start_time)*1000>500):
                choiced_ball=decide(detach_list)
                Trace_ball=Trace(detach_list,frame,choiced_ball)
                # lose_trace_num=0
                start_time=time.perf_counter()
            #对追踪到的球进行处理
            if(Trace_ball!=None):
                #将深度信息转化到车体坐标
                start_time=time.perf_counter()
                base_min_xyz=Trace_ball.base_xyz
                if(Abnormal_judgment(detach_list,frame,Trace_ball,0.19*0.7,0.19*1.5)):
                    r2.Kuang_x=1.0
                    choiced_ball=decide(detach_list)
                    
                else:
                    r2.Kuang_x=0.0
                #滤波
                try:
                    kf.predict()
                    kf.update(float(base_min_xyz[0]))
                    kf2.predict()
                    kf2.update(float(base_min_xyz[1])) #原kalman滤波
                    r2.Chassis_x = float(kf.x)
                    r2.Chassis_y = float(kf2.x)
                except Exception as e:
                    print(e)
                    
                #对画面可视化
                cv2.rectangle(frame,(int(Trace_ball.x1),int(Trace_ball.y1)),(int(Trace_ball.x2),int(Trace_ball.y2)),(0, 255, 0), 2,1) 
                cv2.circle(frame,(int(Trace_ball.xx),int(Trace_ball.yy)), 2, (0, 225, 0), 3)
            else:
                r2.Chassis_x = 0.0
                r2.Chassis_y = 0.0
                r2.Kuang_x=0.0
                kf.predict()
                kf.update(0.0)
                kf2.predict()
                kf2.update(0.0) #原kalman滤波
            cv2.putText(frame, f'FPS: {int(fps)}', (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.imshow('YOLOv8', frame)
            cv2.waitKey(1)
            pub.publish(r2)
            rospy.loginfo("横坐标:%.2f,纵坐标:%.2f", r2.Chassis_x, r2.Chassis_y)
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
            out.write(frame)
    finally:
        
        cv2.destroyAllWindows()
        pipeline.stop()
       

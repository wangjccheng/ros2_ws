import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import cv2

class AIVisionViewer(Node):
    def __init__(self):
        super().__init__('ai_vision_viewer')
        # 监听我们刚才在小脑节点中新增的观测话题
        self.sub = self.create_subscription(
            Float32MultiArray, 
            '/robot/debug/ai_observation', 
            self.obs_callback, 
            10
        )
        # 你的模型高程图尺寸 (441 维对应 21x21)
        self.grid_length = 21 
        self.get_logger().info("👁️ AI 视角可视化器已启动，等待数据...")

    def obs_callback(self, msg):
        obs_data = np.array(msg.data)
        
        # 观测向量总长 467，最后 441 维是高程图
        map_1d = obs_data[- (self.grid_length * self.grid_length):]
        
        # 将一维数组重新 reshape 成 2D 矩阵
        # 这里的 reshape 方式必须和网络内部解析的方式一致
        map_2d = map_1d.reshape(self.grid_length, self.grid_length)

        # ====== 图像渲染准备 ======
        # 将 [-2.0米, 2.0米] 的高度映射到 [0, 255] 的灰度值
        map_norm = np.clip((map_2d + 2.0) / 4.0, 0.0, 1.0)
        map_img_8u = (map_norm * 255).astype(np.uint8)

        # 涂上伪彩色 (热力图：红色代表高凸起，蓝色代表深坑，绿色/黄色是平地)
        map_color = cv2.applyColorMap(map_img_8u, cv2.COLORMAP_JET)

        # 放大图像以便观察 (将 21x21 放大到 420x420)，使用最近邻插值保持像素颗粒感
        map_color_resized = cv2.resize(map_color, (420, 420), interpolation=cv2.INTER_NEAREST)

        # 在图像上添加方向十字线辅助判断
        center = 420 // 2
        cv2.line(map_color_resized, (center, 0), (center, 420), (255, 255, 255), 1) # 竖线
        cv2.line(map_color_resized, (0, center), (420, center), (255, 255, 255), 1) # 横线
        cv2.putText(map_color_resized, "FRONT?", (center - 30, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        cv2.imshow("AI Brain - Elevation Map Viewer", map_color_resized)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    viewer = AIVisionViewer()
    rclpy.spin(viewer)
    viewer.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
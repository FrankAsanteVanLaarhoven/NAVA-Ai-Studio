import asyncio, json, os, aiohttp
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Twist

class DriveBridge(Node):
    def __init__(self):
        super().__init__('univarm_drive_bridge')
        self.declare_parameter('backend_url', os.environ.get('UNIVARM_BACKEND','http://localhost:8080'))
        self.backend = self.get_parameter('backend_url').get_parameter_value().string_value
        self.path_pub = self.create_publisher(Path, '/univarm/drive/path', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.token = 'demo-token-123'
        self.create_timer(0.1, self._noop)
        asyncio.ensure_future(self.run())

    def _noop(self):
        pass

    async def run(self):
        async with aiohttp.ClientSession() as sess:
            payload = {'start':[0,0,0], 'goal':[2,0,0], 'objectives':['shortest']}
            try:
                async with sess.post(f'{self.backend}/api/solve', json=payload) as resp:
                    js = await resp.json()
                    path = Path()
                    path.header.frame_id = 'map'
                    for xy in js.get('path', []):
                        x, y = float(xy[0]), float(xy[1])
                        ps = PoseStamped()
                        ps.header.frame_id = 'map'
                        ps.pose.position.x = x
                        ps.pose.position.y = y
                        path.poses.append(ps)
                    self.path_pub.publish(path)
                    self.get_logger().info(f'Published path with {len(path.poses)} points')
            except Exception as e:
                self.get_logger().error(f'/api/solve error: {e}')

            sse = f'{self.backend}/api/drive/sse?token={self.token}'
            try:
                async with sess.get(sse) as resp:
                    async for line_bytes in resp.content:
                        try:
                            s = line_bytes.decode().strip()
                            if not s.startswith('data:'):
                                continue
                            data = json.loads(s[5:].strip())
                            if 'v' in data and 'w' in data:
                                tw = Twist()
                                tw.linear.x = float(data['v'])
                                tw.angular.z = float(data['w'])
                                self.cmd_pub.publish(tw)
                        except Exception:
                            pass
            except Exception as e:
                self.get_logger().error(f'SSE error: {e}')

def main():
    rclpy.init()
    node = DriveBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

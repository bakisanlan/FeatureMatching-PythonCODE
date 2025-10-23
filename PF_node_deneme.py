# pf_node.py
import rclpy, torch, time
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from queue import Queue, Full, Empty
from concurrent.futures import ThreadPoolExecutor
from std_msgs.msg import Header

class ParticleFilterNode(Node):
    def __init__(self):
        super().__init__('particle_filter')
        self.group = ReentrantCallbackGroup()

        # --- GPU setup (one worker/thread owns the GPU) ---
        torch.set_grad_enabled(False)
        torch.backends.cudnn.benchmark = True
        torch.set_num_threads(1)  # keep CPU threads tame on Jetson
        self.device = torch.device('cuda:0')

        # Load XFeat + LightGlue once, on GPU
        self.xfeat = load_xfeat().to(self.device).eval()
        self.lightglue = load_lightglue().to(self.device).eval()
        warmup(self.xfeat, self.lightglue, self.device)

        # Small “latest only” queue to avoid backlog
        self.in_q = Queue(maxsize=1)

        # Single worker thread to own the GPU
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.pool.submit(self._gpu_worker)

        # I/O
        self.sub = self.create_subscription(
            ImagePairMsg, '/pf/image_pair', self._on_images,
            qos_profile_sensor_data, callback_group=self.group
        )
        self.pub = self.create_publisher(PFResultMsg, '/pf/result', 10)

    def _on_images(self, msg):
        """Fast, non-blocking callback: push latest and return."""
        item = (msg.img0, msg.img1, msg.header)
        try:
            # drop older work if GPU is busy
            if self.in_q.full():
                _ = self.in_q.get_nowait()
            self.in_q.put_nowait(item)
        except Full:
            pass  # already dropped; keep spinning

    def _gpu_worker(self):
        """Runs forever on a dedicated thread, owning CUDA context."""
        stream = torch.cuda.Stream(device=self.device)
        while rclpy.ok():
            try:
                img0, img1, header = self.in_q.get(timeout=0.1)
            except Empty:
                continue
            start = time.time()
            with torch.cuda.stream(stream):
                # Move to GPU (use pinned host memory upstream if possible)
                t0 = to_tensor(img0, self.device, non_blocking=True)
                t1 = to_tensor(img1, self.device, non_blocking=True)

                # XFeat -> features
                kps0, desc0 = self.xfeat(t0)
                kps1, desc1 = self.xfeat(t1)

                # LightGlue -> matches
                matches = self.lightglue(kps0, desc0, kps1, desc1)

                # … run your measurement update, weight update, resampling, etc.
                pf_out = run_measurement_update(matches)

            torch.cuda.synchronize(self.device)  # ensure work is done
            latency = time.time() - start
            self._publish_result(pf_out, header, latency)

    def _publish_result(self, pf_out, header: Header, latency_s: float):
        msg = PFResultMsg()
        msg.header = header
        msg.latency_ms = int(latency_s * 1000)
        # fill the rest of pf_out fields...
        self.pub.publish(msg)

import numpy as np
import multiprocessing as mp
from multiprocessing import shared_memory
import ctypes

class SharedMemoryManager:
    def __init__(self, width=960, height=540):
        self.frame_shape = (height, width, 3)
        self.frame_size = height * width * 3
        self.frame_dtype = np.uint8
        
        # Camera → AI (Triple Buffer)
        self.cam_ai_buffers = self._create_buffers(3, "cam_ai")
        self.cam_write_idx = mp.Value(ctypes.c_int, 0, lock=False)  # Atomic
        self.ai_read_idx = mp.Value(ctypes.c_int, 0, lock=False)    # Atomic
        self.cam_ai_ready = mp.Array(ctypes.c_bool, [False]*3)      # Lock-protected
        
        # AI → WebRTC (Double Buffer)
        self.ai_webrtc_buffers = self._create_buffers(2, "ai_rtc")
        self.ai_write_idx = mp.Value(ctypes.c_int, 0, lock=False)    # Atomic
        self.webrtc_read_idx = mp.Value(ctypes.c_int, 0, lock=False) # Atomic
        self.ai_webrtc_ready = mp.Array(ctypes.c_bool, [False]*2)   # Lock-protected
        
        self.lock = mp.Lock()

    def _create_buffers(self, count, prefix):
        buffers = []
        for i in range(count):
            try:
                shm = shared_memory.SharedMemory(
                    name=f"{prefix}_{i}",
                    create=True,
                    size=self.frame_size
                )
            except FileExistsError:
                shm = shared_memory.SharedMemory(name=f"{prefix}_{i}")
            buffers.append(shm)
        return buffers

    # Camera Interface (Thread-safe)
    def get_camera_buffer(self):
        """Get next available write buffer (non-blocking)"""
        idx = self.cam_write_idx.value
        buf = np.ndarray(self.frame_shape, dtype=self.frame_dtype, 
                        buffer=self.cam_ai_buffers[idx].buf)
        return buf, idx

    def submit_camera_frame(self, idx):
        """Mark frame as ready for AI (atomic)"""
        with self.lock:
            self.cam_ai_ready[idx] = True
            self.cam_write_idx.value = (idx + 1) % 3

    # AI Interface (Process-safe)
    def get_ai_input_frame(self):
        """Get latest camera frame (blocking with timeout)"""
        with self.lock:
            for _ in range(3):  # Check all buffers
                idx = self.ai_read_idx.value
                if self.cam_ai_ready[idx]:
                    frame = np.ndarray(self.frame_shape, dtype=self.frame_dtype,
                                     buffer=self.cam_ai_buffers[idx].buf).copy()
                    self.cam_ai_ready[idx] = False
                    self.ai_read_idx.value = (idx + 1) % 3
                    return frame
                self.ai_read_idx.value = (idx + 1) % 3
            return None

    def get_ai_output_buffer(self):
        """Get buffer for processed frames (non-blocking)"""
        idx = self.ai_write_idx.value
        buf = np.ndarray(self.frame_shape, dtype=self.frame_dtype,
                       buffer=self.ai_webrtc_buffers[idx].buf)
        return buf, idx

    def submit_ai_frame(self, idx):
        """Mark processed frame as ready (atomic)"""
        with self.lock:
            self.ai_webrtc_ready[idx] = True
            self.ai_write_idx.value = (idx + 1) % 2

    # WebRTC Interface (Thread-safe)
    def get_webrtc_frame(self):
        """Get latest frame for streaming (non-blocking)"""
        idx = self.webrtc_read_idx.value
        if self.ai_webrtc_ready[idx]:
            frame = np.ndarray(self.frame_shape, dtype=self.frame_dtype,
                             buffer=self.ai_webrtc_buffers[idx].buf).copy()
            self.ai_webrtc_ready[idx] = False
            self.webrtc_read_idx.value = (idx + 1) % 2
            return frame
        return None

    def cleanup(self):
        for buf in self.cam_ai_buffers + self.ai_webrtc_buffers:
            buf.close()
            try:
                buf.unlink()
            except:
                pass
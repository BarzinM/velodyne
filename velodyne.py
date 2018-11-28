import socket
import struct
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from threading import Thread, Lock


VERTICAL_ANGLES = np.array([-15, 1, -13, -3, -11, 5, -9, 7,
                            -7, 9, -5, 11, -3, 13, -1, 15]) * np.pi / 180.
ORDER = [15, 13, 11, 9, 7, 5, 3, 1, 14, 12, 10, 8, 6, 4, 2, 0]
C_VER = np.cos(VERTICAL_ANGLES)
S_VER = np.sin(VERTICAL_ANGLES)
AZ = np.arange(0, 360, 0.4) * np.pi / 180.
S_AZ = np.sin(AZ)
C_AZ = np.cos(AZ)
TO_X = C_VER[:, None] * S_AZ
TO_Y = C_VER[:, None] * C_AZ


def check_block(b):
    return b[:2] == b'\xff\xee'


def get_azimuth_hundredth_degree(b):
    return struct.unpack("<H", b[2:4])[0]


def get_distances(b):
    fmt = "<" + ("Hc" * 32)
    data = struct.unpack(fmt, b[4:])[::2]
    return data


class Velodyne(object):

    def __init__(self):
        self.az = np.zeros(12, dtype=np.int)
        self.dist = np.zeros((16, 12), dtype=np.int)
        self._scene = np.zeros((16, int(360 * 2.5)), dtype=np.int)

        # UDP_IP = "192.168.1.201"
        UDP_PORT = 2368
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("", UDP_PORT))

    def begin(self):
        self.running = True
        self.thread = Thread(target=self._update)
        self.thread.daemon = True
        self.thread.start()
        sleep(.2)

    def _update(self):
        while self.running:
            data, addr = self.sock.recvfrom(1248)
            if addr[1] != 2368:
                continue
            if len(data) < 1206:
                raise
            blocks = [data[i * 100:(i + 1) * 100] for i in range(12)]
            # timestamp = data[1200:1204]
            # factory = data[1204:1206]
            for i in range(12):
                if not check_block(blocks[i]):
                    raise
                self.az[i] = get_azimuth_hundredth_degree(blocks[i])
                self.dist[:, i] = get_distances(blocks[i])[:16]

            indx = self.az * .025
            indx = (450 + np.rint(indx).astype(int)) % 900
            self._scene[:, indx] = self.dist

    def close(self):
        self.running = False
        self.thread.join()
        print(" Closed Lidar Device.")

    @property
    def scene(self):
        return self._scene[ORDER] * .002

    @property
    def scene_raw(self):
        return self._scene[ORDER]

    @property
    def xyz(self):
        s = self._scene * .002
        x = np.multiply(s, TO_X)
        y = np.multiply(s, TO_Y)
        z = S_VER[:, None] * s
        # return x, y, z
        return x.flatten(), y.flatten(), z.flatten()

    def plot_2d(self):
        plt.imshow(self.scene, aspect='auto')
        plt.show()

    def plot_3d(self):
        fig = plt.figure()
        ax = Axes3D(fig)
        x, y, z = self.xyz

        ax.scatter(x, y, z, marker=".")
        plt.show()

    def live_2d(self, refresh_rate=.1, figsize=None):
        plt.ion()
        fig, ax = plt.subplots(1, 1, figsize=figsize)
        img_handle = ax.imshow(self.scene, aspect='auto')
        while True:
            sleep(refresh_rate)
            img_handle.set_data(self.scene)
            fig.canvas.draw()

    def live_3d(self, refresh_rate=.1, figsize=None):
        import matplotlib.animation as anim
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        x, y, z = self.xyz
        graph = ax.plot(x, y, z, linestyle="", marker=".")[0]

        def update_graph(*args):
            x, y, z = self.xyz
            graph.set_data(x, y)
            graph.set_3d_properties(z)

        do_not_remove = anim.FuncAnimation(fig, update_graph, None,
                                           interval=40, blit=False)
        plt.show()


if __name__ == "__main__":
    from time import sleep
    try:
        v = Velodyne()
        v.begin()

        s = v.scene[:-3]
        mask = np.logical_and(0 < s, s < .8)
        obstacle = np.sum(mask) > 30

        print(obstacle, np.sum(mask))

        plt.imshow(mask, aspect='auto')
        fig = plt.figure()
        plt.imshow(s, aspect='auto')
        plt.show()

    finally:
        v.close()

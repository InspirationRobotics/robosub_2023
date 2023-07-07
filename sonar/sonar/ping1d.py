import brping


class Ping1D(brping.Ping1D):
    def __init__(self, device, baudrate=115200):
        super(Ping1D, self).__init__()
        self.connect_serial(device, baudrate)

        if not self.initialize():
            raise RuntimeError("failed to initialize Ping1D")

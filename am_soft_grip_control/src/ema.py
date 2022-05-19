# exponential moving average
class EMA:
    def __init__(self, alpha):
        self.previous_in = None
        self.alpha = alpha
        self.avg = 0

    def average(self, data_in):
        if self.previous_in is None:
            self.avg = data_in
        else:
            self.avg = self.alpha * data_in + (1 - self.alpha) * self.avg
        self.previous_in = data_in
        return self.avg
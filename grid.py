from math import log

class Grid:
    def __init__(self):
        #initialization
        self.log_odds = 0.5

    def set_lo(self,lo):
        self.log_odds = lo

    def get_lo(self):
        return self.log_odds

    def update_lo(self,log_likelihood):
        self.log_odds += log_likelihood

from abc import ABC, abstractmethod
import numpy as np

def nt_round(x, prec=2, base=.25):
    return round(base * round(float(x)/base), prec)

class TestResults(object):
    def __init__(self, total_pts:int, precision=0.25):
        self.names = []
        self.weights = []
        self.results = []
        self.total_pts = total_pts
        self.precision = precision

    def add_result(self, passed:bool, *, weight=1, name=''):
        self.results.append(1 if passed else 0) # explicit better than implicit
        self.weights.append(weight)
        self.names.append(name)

    def points(self):
        assert len(self.results) > 0
        # print(f'points: {self.results} {self.weights} {self.total_pts} {self.weights}')
        pts = np.dot(np.array(self.results), np.array(self.weights))
        pts *= self.total_pts / float(np.sum(self.weights))
        return nt_round(pts, base=self.precision)

    def __str__(self):
        max_names = 2 + np.max([len(n) for n in self.names])
        ret = ''
        for n, w, r in zip(self.names, self.weights, self.results):
            ret += n.ljust(max_names)
            ret += f'{r*w:02d}/{w:02d}'
            ret += '\n'
        return ret

class TestBase(ABC):
    PTS = None

    def __init__(self):
        pass

    '''
    test: run the test
    Should report tuppe (weights, passes). Both weights and passes are arrays
    '''
    @abstractmethod
    def do_test(self, module, result):
        pass

    def report_total_points(self):
        '''
        Note: the abbrivation "pts" is used for variable and the full name "points" is used in functions
        '''
        return self.PTS

    def test(self, module):
        result = TestResults(self.report_total_points())
        self.do_test(module, result)
        return result

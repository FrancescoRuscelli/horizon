from scipy.io import savemat, loadmat
import numpy as np

class matStorer:
    def __init__(self, file_name):
        self.file_name = file_name
        self.dict_values = dict()

    # def append(self, values):
    #     with open(self.file_name, 'ab') as f:
    #         savemat(f, {'pad': values})

    def append(self, dict_values):
        self.dict_values.update(dict_values)

    def store(self, dict_values):
        self.append(dict_values)
        savemat(self.file_name, self.dict_values)  # write

    def save(self):
        savemat(self.file_name, self.dict_values)

    def load(self):
        return loadmat(self.file_name)

if __name__ == '__main__':
    a = np.ones([1,5])
    b = 2* np.ones([1,5])
    filename = 'try.mat'
    ms = matStorer(filename)
    ms.store({'a': a})
    mat = ms.load()
    print(mat)
    # mat = loadmat(filename)
    # print(mat)



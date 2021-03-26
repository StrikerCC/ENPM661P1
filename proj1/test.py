import numpy as np

a = np.array([[1, 2, 3, 4], [5, 6, 0, 8], [9, 10, 7, 12], [13, 14, 11, 15]])

state = np.transpose(a).flatten()
for i in state:
    print(i)
print(str(state))

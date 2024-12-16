import numpy as np

frequency = 456 # mHz
amplitude = 0.9
numberOfSteps = int ((1/(frequency/1000)) * 50) # 512 steps = 5.12 s
if numberOfSteps > 512:
    numberOfSteps = 512
elif numberOfSteps == 0:
    print('1\n{0x3FF}')
else:
    print(numberOfSteps)
    x = ((amplitude*np.cos(np.linspace(1.0*np.pi, 3.0*np.pi, num=numberOfSteps)))+1)*511.5
    outPut = x.astype(int)
    print('{', end='')
    for i in range(numberOfSteps):
        print('0x'+"{:03x}".format(outPut[i]), end='')
        if i != numberOfSteps-1:
            print(', ', end='')
    # print(outPut, end='')
    print('}')
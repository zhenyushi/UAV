#!/usr/bin/env python
import matplotlib.pyplot as plt
from scipy import signal
import numpy as np

if __name__ == '__main__':

    x_start = 0
    x_end = 1

    y_start = 2
    y_end = -2
    
    plt.figure(figsize=(16,12))

    t = np.linspace(0, 1, 500, endpoint=False)
    plt.subplot(2, 2, 1)
    plt.plot(t, signal.square(2 * np.pi * 5 * t))
    
    plt.xlim(x_start, x_end)
    plt.ylim(y_start, y_end)

    sig = np.sin(2 * np.pi * t)
    pwm = signal.square(2 * np.pi * 30 * t, duty=(sig + 1)/2)
    plt.subplot(2, 2, 2)
    plt.plot(t, sig)
    plt.subplot(2, 2, 3)
    plt.plot(t, pwm)
    plt.ylim(-1.5, 1.5)
    plt.show()

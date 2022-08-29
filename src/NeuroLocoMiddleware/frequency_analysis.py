import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import fft, fftfreq


def init_bode_plot():
    fig, axs = plt.subplots(2,1, sharex=True, figsize=(16,8))

    axs[0].set_ylabel('Magnitude')
    axs[1].set_ylabel('Phase, deg')
    axs[1].set_yticks([-180,-90,0,90,180])
    axs[1].set_xlabel("Frequency, Hz")
    return fig, axs



def fft_bode_plot(t_data, y_data, u_data, axs=None, **kwargs):
    if axs is None:
        fig, axs = init_bode_plot()
    N = t_data.shape[0]
    T = t_data[-1]/N
    yf = fft(y_data)[0:N//2]
    uf = fft(u_data)[0:N//2]
    xf = fftfreq(N, T)[:N//2]
    tf = yf*uf.conjugate() / (uf*uf.conjugate())
    # plt.plot(xf, 2.0/N * np.abs(yf[0:N//2]))
    axs[0].loglog(xf, np.abs(tf), **kwargs)
    axs[1].semilogx(xf, 180/np.pi*np.angle(tf), **kwargs)
    axs[0].set_ylabel('Magnitude')
    axs[1].set_ylabel('Phase, deg')
    axs[1].set_yticks([-180,-90,0,90,180])
    axs[1].set_xlabel("Frequency, Hz")


def tf_bode_plot(freqs, tf, axs=None, **kwargs):
    if axs is None:
        fig, axs = init_bode_plot()
    s = complex(0,1)*freqs*2*np.pi

    axs[0].loglog(freqs, np.abs(tf(s)), **kwargs)
    axs[1].semilogx(freqs, 180/np.pi*np.angle(tf(s)), **kwargs)
    axs[0].set_ylabel('Magnitude')
    axs[1].set_ylabel('Phase, deg')
    axs[1].set_yticks([-180,-90,0,90,180])
    axs[1].set_xlabel("Frequency, Hz")
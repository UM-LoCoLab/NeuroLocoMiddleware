from FindLibrariesWarning import *
import numpy as np
import matplotlib.pyplot as plt
import LinearFilter as lf
import SysID as sid
import frequency_analysis as fa



def run_test(times, chirp, system, axs, **kwargs):
	N = len(times)
	us, ys = np.zeros((N,)), np.zeros((N,))
	for i, t in enumerate(times):
		u = chirp.next(t)
		y = system.next(u)
		us[i]=u
		ys[i]=y
	fa.fft_bode_plot(times, ys, us, axs=axs, **kwargs)
	axs[-1].set_xlim([1,100])
	fig = plt.figure()
	plt.plot(times, us)
	plt.plot(times, ys)


def plot_biquad():
	print(dir(lf))
	print(dir(sid))
	print(dir(fa))
	fig,axs = fa.init_bode_plot()
	T = 100
	test_chirp = sid.Chirp(.3, 30, T)
	test_system = lf.BiQuad(7,0.2, 7,0.9)
	fa.tf_bode_plot(np.logspace(0.1,3,1000), 
		lambda s: (2*np.pi*7)**2/(2*np.pi*7)**2 * (s**2 + 2*0.2*(2*np.pi)*7*s + (2*np.pi*7)**2)/(s**2 + 2*0.9*(2*np.pi*7)*s + (2*np.pi*7)**2),
		 axs=axs, color='k' )
	N = 300*T
	times = np.linspace(0,T,N)
	test_system.discretize_taylor_3(times[1]-times[0])
	# us, ys = np.zeros((N,)), np.zeros((N,))
	# for i, t in enumerate(times):
	# 	u = test_chirp.next(t)
	# 	y = test_system.next(u)
	# 	us[i]=u
	# 	ys[i]=y
	# fa.fft_bode_plot(times, ys, us, axs=axs)
	# axs[-1].set_xlim([1,100])
	run_test(times, test_chirp, test_system, axs, label='taylor')
	test_system.discretize_substep_euler(times[1]-times[0], N=10)
	run_test(times, test_chirp, test_system, axs, label='euler, N=10')
	test_system.discretize_substep_euler(times[1]-times[0], N=1000)
	run_test(times, test_chirp, test_system, axs, label="euler, N=1000")
	test_system.discretize_tustin(times[1]-times[0])
	run_test(times, test_chirp, test_system, axs, label="tustin")
	test_system.discretize_N_tustin(times[1]-times[0], N=10)
	run_test(times, test_chirp, test_system, axs, label="tustin, N=10")
	test_system.discretize_N_tustin(times[1]-times[0], N=100)
	run_test(times, test_chirp, test_system, axs, label="tustin, N=100")
	test_system = lf.DiscreteBiQuad(7,0.2, 7,0.9, times[1]-times[0])
	run_test(times, test_chirp, test_system, axs, label="tustin, N=100")
	axs[0].legend()
	plt.show()


def main():
	plot_biquad()

if __name__ == '__main__':
	main()
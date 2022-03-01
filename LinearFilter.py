import numpy as np
from math import exp, log, sqrt

class LinearFilter():

	""" A linear filter, state space, re-discretizable
	"""
	
	def __init__(self, A, B, C, D, x0, dt=1/100):
		"""Summary
		
		Args:
			A (np matrix): state transition matrix
			B (np matrix): input matrix
			C (np matrix): measurement matrix
			D (np matrix): direct input matrix
			X0 (np matrix): initial filter states
			dt (float, optional): Time step
		
		"""
		self.A = np.array(A)
		self.B = np.array(B)
		self.C = np.array(C)
		self.D = np.array(D)
		self.dt = dt
		self.discretize_taylor_3(self.dt)
		self.Cd = C
		self.Dd = D
		self.x = np.array(x0) # avoids side effects
		self.y = self.x

	def discretize_taylor_3(self,dt):
		"""Discretizes the continuous state space system using the Taylor series
		approximation of the matrix exponential
		
		Args:
			dt (float): the time step
		written by Leo
		"""
		self.dt = dt
		self.Ad = np.eye(self.A.shape[0]) + self.A * dt + (0.5 * self.A @ self.A * dt**2) + (1/6 * self.A @ self.A @ self.A* dt**3)
		self.Bd = (self.Ad - np.eye(self.A.shape[0])) @ np.linalg.solve(self.A,self.B)

	def discretize_tustin(self,dt):
		self.dt = dt

		# z = (ω₀ + s) / (ω₀ - s) 
		# (ω₀ - s) z = (ω₀ + s)
		# ω₀ z - ω₀ = s + s z
		# s = ω₀(z-1)/(z+1)

		# s I x = A x + B u
		# y = C x + Du
		# s = 2/T * (z-1)/(z+1)
		# 2/T * (z-1) x/(z+1) =  A x + B u
		# ξ(z+1) = 2 x
		# 1/T * (z-1) ξ =  A 1/2 ξ(z+1)  + B u
		# (z-1) ξ = T A 1/2 ξ(z+1)  + T B u
		# z ξ - ξ = T A 1/2 zξ + T A 1/2 ξ  + T B u
		# (I - T A 1/2) z ξ  = (I + T A 1/2) ξ  + T B u
		# z ξ  = (I - T A 1/2)\(I + T A 1/2) ξ  + (I - T A 1/2)\T B u
		# y = Cx + Du 
		# y = C (z+1)/2 ξ + Du
		# y = C/2 z ξ  + C/2 ξ + Du
		# y = C/2 [(I - T A 1/2)\(I + T A 1/2) ξ  + (I - T A 1/2)\T B u]  + C/2 ξ + Du
		# y = C/2 ((I - T A 1/2)\(I + T A 1/2)+I) ξ  + (D + C/2 (I - T A 1/2)\T B)u 

		I = np.eye(self.A.shape[0])
		print("tustin, eig A", np.linalg.eig(self.A))
		self.Ad = np.linalg.solve(I - 0.5*dt*self.A, I + 0.5*dt*self.A)
		print("tustin, eig Ad", np.linalg.eig(self.Ad))
		self.Bd = np.linalg.solve(I - 0.5*dt*self.A,  dt*self.B)
		self.Cd = 0.5*self.C @ (I+self.Ad)
		self.Dd = self.D + 0.5*self.C@self.Bd
		assert(self.Ad.shape==self.A.shape)
		assert(self.Bd.shape==self.B.shape)
		assert(self.Cd.shape==self.C.shape)
		assert(self.Dd.shape==self.D.shape)

	def discretize_N_tustin(self, dt, N=5):
		self.discretize_tustin(dt/N)
		A = self.Ad
		B = self.Bd
		for i in range(N):
			self.Ad = A@self.Ad
			self.Bd = A@self.Bd + B

	def discretize_substep_euler(self, dt, N=5):
		self.Ad = np.eye(self.A.shape[0])
		self.Bd = np.zeros(self.B.shape)
		sub_dt = (dt/N)
		for i in range(N):
			self.Ad = (np.eye(self.A.shape[0])+self.A*sub_dt)@self.Ad
			self.Bd = (np.eye(self.A.shape[0])+self.A*sub_dt)@self.Bd+sub_dt*self.B

	def chain(self, filtB):
		""" returns convolution of self * filtB """
		N = self.A.shape[0]+filtB.A.shape[0]
		Nx1 = self.A.shape[0]
		Nx2 = filtB.A.shape[0]
		A = np.zeros((N,N))
		A[:Nx1,:Nx1] = self.A
		A[Nx1:,Nx1:] = filtB.A
		A[:Nx1,Nx1:] = self.B@filtB.C
		B = np.zeros((N,filtB.B.shape[1]))
		B[:Nx1,:] = self.B@filtB.D
		B[Nx1:,:] = filtB.B
		C = np.zeros((self.C.shape[0],N))
		C[:, :Nx1] = self.C
		C[:, Nx1:] = self.D@filtB.C
		D = self.D@filtB.D
		x0 = np.vstack([self.x, filtB.x])
		return LinearFilter(A, B, C, D, x0, dt=self.dt)

	def next(self, u):
		try:
			u = u.reshape(-1,1)
		except :
			u = np.array([[u]])

		assert(self.x.shape[0] == self.Ad.shape[1])
		assert(u.shape[0] == self.Bd.shape[1])

		self.x = (self.Ad @ self.x + self.Bd @ u)
		self.y = self.Cd @ self.x + self.Dd @ u

		return self.y

def from_poly_fraction(polyNum, polyDen, dt=0.01):
	""" returns (n0+n1 s+ n2 s^2 +...) / (d0 + d1 s + d2 s^2 + ...)
	requires len(polyNum)<=len(polyDen)
	"""
	assert(len(polyNum)<=len(polyDen))
	nx = len(polyDen)-1
	A = np.zeros((nx,nx))
	A[:-1,1:]=np.eye(nx-1)
	A[-1,:] = -np.array(polyDen)[:-1]/polyDen[-1]
	B = np.zeros((nx,1))
	B[-1,0] = 1/polyDen[-1]
	# x = 1 / (d0 + d1 s + d2 s^2 + ...) u
	C = np.zeros((1,nx))
	for i in range(nx):
		C[0,i]=polyNum[i]
	if len(polyNum)==len(polyDen):
		d = polyNum[-1]/polyDen[-1]
		D = np.array([[d]])
		C += d*A[-1:,:]*polyDen[-1]
		# y = nq/dq u + (n0 - d0 nq/dq + (n1 -d1 nq/dq)s +...+ (nq - dq nq/dq)s^q)/(d0 + d1 s +...+ dq s^q)u
	x0 = np.zeros((nx,1))
	return LinearFilter(A,B,C,D,x0,dt=dt)

def BiQuad(ωn,ζn, ωd,ζd):
	ωn*=2*np.pi
	ωd*=2*np.pi
	return from_poly_fraction([ωn**2, 2*ωn*ζn, 1],[ωd**2, 2*ωd*ζd, 1])

def DiscreteBiQuad(ωn,ζn, ωd,ζd, dt):
	ωn*=2*np.pi
	ωd*=2*np.pi

	c_z1 = complex(- ωn*ζn, ωn*sqrt(1-ζn**2))
	c_z2 = complex(- ωn*ζn, -ωn*sqrt(1-ζn**2))
	c_p1 = complex(- ωd*ζd, ωd*sqrt(1-ζd**2))
	c_p2 = complex(- ωd*ζd, -ωd*sqrt(1-ζd**2))


	d_z1 = exp(dt*c_z1)
	d_z2 = exp(dt*c_z2)
	d_p1 = exp(dt*c_p1)
	d_p2 = exp(dt*c_p2)




	sys = from_poly_fraction([(d_z1*d_z2).real, -(d_z1+d_z2).real, 1],[(d_p1*d_p2).real, -(d_p1+d_p2).real, 1])
	sys.Ad = sys.A
	sys.Bd = sys.B
	sys.Cd = sys.C
	sys.Dd = sys.D
	return sys

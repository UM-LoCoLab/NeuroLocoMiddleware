import numpy as np

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
		assert(order>=1) # linear is lowest possible order
		self.dt = dt
		self.Ad = np.eye(self.A.shape[0]) + self.A * dt + (0.5 * self.A @ self.A * dt**2) + (1/6 * self.A @ self.A @ self.A* dt**3)
		self.Bd = (self.Ad - np.eye(self.A.shape[0])) @ np.linalg.solve(self.A,self.B)

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
	nx = len(polyDen)
	A = np.zeros((nx,nx))
	A[:-1,1:]=np.eye(nx-1)
	A[-1,:] = -np.array(polyDen)[:-1]/polyDen[-1]
	B = np.zeros((nx,1))
	B[-1,0] = 1/polyDen[-1]
	# x = 1 / (d0 + d1 s + d2 s^2 + ...) u
	C = np.zeros((1,nx))
	for i in range(len(polyNum)):
		C[0,i]=polyNum[i]
	if len(polyNum)==len(polyDen):
		d = polyNum[-1]/polyDen[-1]
		D = np.array([[d]])
		C += d*A[-1:,:]*polyDen[-1]
		# y = nq/dq u + (n0 - d0 nq/dq + (n1 -d1 nq/dq)s +...+ (nq - dq nq/dq)s^q)/(d0 + d1 s +...+ dq s^q)u
	x0 = np.zeros((nx,1))
	return LinearFilter(A,B,C,D,x0,dt=dt)

def BiQuad(ωn,ζn, ωd,ζd):
	return from_poly_fraction([ωn**2, 2*ωn*ζn, 1],[ωd**2, 2*ωd*ζd, 1])

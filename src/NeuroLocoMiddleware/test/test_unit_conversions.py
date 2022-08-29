import numpy as np
MOTOR_CLICKS_PER_REVOLUTION = 16384 
NM_PER_AMP = 0.146
RAD_PER_CLICK = 2*np.pi/MOTOR_CLICKS_PER_REVOLUTION
RAD_PER_DEG = np.pi/180.
K=300
B=1600

A = 0.00028444
C = 0.0007812
Nm_per_rad_to_Kunit = RAD_PER_CLICK/C*1e3/NM_PER_AMP
Nm_s_per_rad_to_Bunit = RAD_PER_DEG/A*1e3/NM_PER_AMP

print(K/Nm_per_rad_to_Kunit)
print(B/Nm_s_per_rad_to_Bunit)
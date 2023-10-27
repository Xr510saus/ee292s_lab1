
import numpy as np

from pprint import pprint
import allantools
import matplotlib.pyplot as plt


# data = np.random.randint(np.arange(0, samples), samples) # random int array for testing

accelXdata = np.genfromtxt('data.csv',
                     dtype=float,
                     delimiter=',',
                     usecols=(0,)
                    )

accelYdata = np.genfromtxt('data.csv',
                     dtype=float,
                     delimiter=',',
                     usecols=(1,)
                    )

accelZdata = np.genfromtxt('data.csv',
                     dtype=float,
                     delimiter=',',
                     usecols=(2,)
                    )

gyroXdata = np.genfromtxt('data.csv',
                     dtype=float,
                     delimiter=',',
                     usecols=(3,)
                    )

gyroYdata = np.genfromtxt('data.csv',
                     dtype=float,
                     delimiter=',',
                     usecols=(4,)
                    )

gyroZdata = np.genfromtxt('data.csv',
                     dtype=float,
                     delimiter=',',
                     usecols=(5,)
                    )

taus_aX, adev_aX, _, _ = allantools.adev(accelXdata,rate=10, data_type='freq')
plt.figure()
plt.loglog(taus_aX, adev_aX)
plt.xlabel("Tau (s)")
plt.ylabel("Allan Dev")
plt.grid(True, which='both')
plt.legend()
plt.title('AccelX Allan Deviation Plot')

taus_aY, adev_aY, _, _ = allantools.adev(accelYdata,rate=10, data_type='freq')
plt.figure()
plt.loglog(taus_aY, adev_aY)
plt.xlabel("Tau (s)")
plt.ylabel("Allan Dev")
plt.grid(True, which='both')
plt.legend()
plt.title('AccelY Allan Deviation Plot')

taus_aZ, adev_aZ, _, _ = allantools.adev(accelZdata,rate=10, data_type='freq')
plt.figure()
plt.loglog(taus_aZ, adev_aZ)
plt.xlabel("Tau (s)")
plt.ylabel("Allan Dev")
plt.grid(True, which='both')
plt.legend()
plt.title('AccelZ Allan Deviation Plot')

taus_gX, adev_gX, _, _ = allantools.adev(gyroXdata,rate=10, data_type='freq')
plt.figure()
plt.loglog(taus_gX, adev_gX)
plt.xlabel("Tau (s)")
plt.ylabel("Allan Dev")
plt.grid(True, which='both')
plt.legend()
plt.title('GyroX Allan Deviation Plot')

taus_gY, adev_gY, _, _ = allantools.adev(gyroYdata,rate=10, data_type='freq')
plt.figure()
plt.loglog(taus_gY, adev_gY)
plt.xlabel("Tau (s)")
plt.ylabel("Allan Dev")
plt.grid(True, which='both')
plt.legend()
plt.title('GyroY Allan Deviation Plot')

taus_gZ, adev_gZ, _, _ = allantools.adev(gyroZdata,rate=10, data_type='freq')
plt.figure()
plt.loglog(taus_gZ, adev_gZ)
plt.xlabel("Tau (s)")
plt.ylabel("Allan Dev")
plt.grid(True, which='both')
plt.legend()
plt.title('GyroZ Allan Deviation Plot')

plt.show()

# print(data[:10])

# samples = len(accelXdata[0:90000])
# # print(samples)
# tau = 10 # cluster size

# K = samples / tau # number of clusters
# # print(K)

# # print(data[-1])
# chunks = np.split(accelXdata[0:90000], K)  # split data into K clusters
# chunk_means = []

# for chunk in chunks:
#     chunk_mean = np.mean(chunk)
#     chunk_means.append(chunk_mean)

# # print(chunks[-1])

# mean_diff_sq = []
# allan_var = []
# running_sum = 0
# for k in range(len(chunk_means)-1):
#     mean_diff_sq.append((chunk_means[k+1] - chunk_means[k])**2)
#     running_sum += mean_diff_sq[k]
#     if k > 0:
#         allan_var.append(1/(2*k) * running_sum)

    # if k == len(chunk_means)-3:
        # print(chunk_means[k+1])
# pprint(mean_diff_sq)
# allan_var = np.sum(mean_diff_sq) * 1/(2*(K-1))
# allan_dev = np.sqrt(allan_var)

# plt.figure()
# plt.loglog([x for x in range(len(chunks)-2)],allan_dev)
# plt.xlabel("Tau (s)")
# plt.ylabel("Allan Dev")
# plt.grid(True, which='both')
# plt.legend()
# plt.title('AccelX Allan Deviation Plot')
# plt.show()

# # pprint(chunks)
# # pprint(len(chunk_means))
# pprint(allan_var)
# pprint(allan_dev)
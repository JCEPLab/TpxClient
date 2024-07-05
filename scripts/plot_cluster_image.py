from cluster_file_utils import ClusterFileIterator, num_clusters
from tkinter.filedialog import askopenfilenames
from fast_histogram import histogram2d

import numpy as np
import matplotlib.pyplot as plt

files = askopenfilenames(filetypes=[("Binary cluster files", "*.clusters"), ])

for file in files:
    count = num_clusters(file)
    x = np.empty((count,1))
    y = np.empty((count,1))

    for ix, cluster in enumerate(ClusterFileIterator(file)):
        x[ix] = cluster[0]
        y[ix] = cluster[1]

    image = histogram2d(x,y,range=((0,256),(0,256)),bins=256)

    plt.figure()
    plt.imshow(image)

plt.show()
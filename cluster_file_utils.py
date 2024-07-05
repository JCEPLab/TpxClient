import os

def clusters_to_csv(input_files,  output_files=None):
    if type(input_files) not in (list, tuple):
        input_files = [input_files,]

    if output_files is None:
        output_files = [f + '.csv' for f in input_files]

    if len(input_files) != len(output_files):
        raise RuntimeError(f"clusters_to_csv(): input and output files have different lengths ({len(input_files)} vs. {len(output_files)})")

    for fix in range(len(input_files)):
        file = input_files[fix]
        with open(output_files[fix], 'w') as fout:
            fout.write("X [pixel],Y [pixel],T [1.5625ns]\n")
            for cluster in ClusterFileIterator(file):
                fout.write(f"{cluster[0]},{cluster[1]},{cluster[2]}\n")

class ClusterFileIterator:

    def __init__(self, fname:str):
        self._file = open(fname, 'rb')
        validate_file_header(fname)
        self._file.read(8) # skip the header
        self._num_clusters = num_clusters(fname)

    def __iter__(self):
        return self

    def __next__(self):
        data = self._file.read(8)
        if len(data) < 8:
            raise StopIteration

        x = int(data[0])
        y = int(data[1])
        t = int.from_bytes(data[2:], byteorder='big') & 0x3FFFFFFFFF

        return (x,y,t)

    def numClusters(self):
        return self._num_clusters

def validate_file_header(fname:str):
    with open(fname, 'rb') as fin:
        header = fin.read(8)
        if header != b'CLUSTERS':
            raise RuntimeError(f"Invalid cluster file header; expected b'CLUSTERS', found {header}")

def num_clusters(fname:str):
    validate_file_header(fname)
    return os.stat(fname).st_size//8 - 1
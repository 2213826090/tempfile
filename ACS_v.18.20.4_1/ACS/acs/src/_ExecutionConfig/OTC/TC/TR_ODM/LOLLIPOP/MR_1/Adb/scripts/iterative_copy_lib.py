import os

def gen_random_binary_file(name, size):
    with open(name, 'wb') as fout:
        fout.write(os.urandom(size))

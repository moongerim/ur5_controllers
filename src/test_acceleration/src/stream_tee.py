import sys
import datetime
import scipy.io
import os

def generate_timestamp():
    return datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
def write_mat(dir, data_dict,filename):
    filename=filename+'.mat'
    timestamp = datetime.datetime.now()
    str_time = timestamp.strftime('_%H_%M')
    name='experiment'+str_time
    folder_name = dir
    if not os.path.exists(folder_name):
        os.makedirs(folder_name)
    filename=os.path.join(folder_name, filename)
    with open(filename, 'wb') as f:
        scipy.io.savemat(f, data_dict)
    print("Printed .mat files in "+folder_name)

class stream_tee(object):
    # Based on https://gist.github.com/327585 by Anand Kunal
    def __init__(self, stream1, stream2):
        self.stream1 = stream1
        self.stream2 = stream2
        self.__missing_method_name = None  # Hack!

    def __getattribute__(self, name):
        return object.__getattribute__(self, name)

    def __getattr__(self, name):
        self.__missing_method_name = name  # Could also be a property
        return getattr(self, '__methodmissing__')

    def __methodmissing__(self, *args, **kwargs):
        # Emit method call to the log copy
        callable2 = getattr(self.stream2, self.__missing_method_name)
        callable2(*args, **kwargs)

        # Emit method call to stdout (stream 1)
        callable1 = getattr(self.stream1, self.__missing_method_name)
        return callable1(*args, **kwargs)


if __name__ == '__main__':
    logfile = file("blah.txt", "w+")

    sys.stdout = stream_tee(sys.stdout, logfile)

    print(generate_timestamp())
    print("# Now, every operation on sys.stdout is also mirrored on logfile")

    logfile.close()

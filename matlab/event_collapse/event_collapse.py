import re
import numpy as np

pattern = re.compile('(\d+) (\d+\.\d+) AE \((.*)\)')

infile = '/home/miacono/workspace/DATASETS/mapping_rescaled/2/ATIS/data.log'
outfile = '/home/miacono/workspace/DATASETS/mapping_rescaled/2/ATIS/data_collapsed.log'
def pretty_print_AE(bottlenumber, timestamp, events):
    return '%d %f AE (%s)' %(bottlenumber, timestamp,
                             ' '.join(map(str, list(events)))
)

def something(filename):
    with open(filename, 'r') as f:
        content = f.read()[:1000000]
        found = pattern.findall(content)
        buffer = []
        first_timestamp = None
        for elem in found:

            events = np.int32(elem[2].split())
            bottlenumber = np.int32(elem[0])
            timestamp = np.float64(elem[1])
            #print(pretty_print_AE(bottlenumber, timestamp, events))
            buffer.append(events)
            if first_timestamp is None:
                first_timestamp = timestamp
            print (timestamp - first_timestamp)
            if (timestamp-first_timestamp > 2e-3):
                print("whatever")
                first_timestamp = timestamp
                pp = pretty_print_AE(bottlenumber, timestamp,
                                      np.concatenate(buffer))
                buffer = []
                yield pp

if __name__ == '__main__':
    text_out = '\n'.join([line for line in something(infile)])
    with open(outfile, 'w') as of:
        of.write(text_out)
